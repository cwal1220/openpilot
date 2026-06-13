import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import copy_to_f64, interp, interp_to_f64
from common.realtime import DT_MDL
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog
from common.params import Params
from decimal import Decimal

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera. Empirically 
# the model knows the difference between TICI and EON
# so a path offset is not needed
PATH_OFFSET = -(float(Decimal(Params().get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))  # default 0.0
if EON:
  CAMERA_OFFSET = -(float(Decimal(Params().get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))  # m from center car to camera
  CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15
elif TICI:
  CAMERA_OFFSET = 0.04
  CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15
else:
  CAMERA_OFFSET = 0.0
CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15

WIDTH_PROB_BP = (4.0, 5.0)
WIDTH_PROB_V = (1.0, 0.0)
STD_PROB_BP = (0.15, 0.3)
STD_PROB_V = (1.0, 0.0)
SPEED_OFFSET_BP = (0.0, 11.1, 16.6, 22.2, 31.0)
SPEED_OFFSET_V = (0.10, 0.05, 0.02, 0.01, 0.0)


def interp_mono_to_f64(x, xp, fp, out, count):
  for i in range(1, len(xp)):
    if xp[i] < xp[i - 1]:
      out[:count] = np.interp(x[:count], xp, fp)
      return
  interp_to_f64(x, xp, fp, out, count)


class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self._ll_t_right = np.zeros((TRAJECTORY_SIZE,))
    self.width_pts = np.zeros((TRAJECTORY_SIZE,))
    self.path_from_left_lane = np.zeros((TRAJECTORY_SIZE,))
    self.path_from_right_lane = np.zeros((TRAJECTORY_SIZE,))
    self.lane_path_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_path_y_interp = np.zeros((TRAJECTORY_SIZE,))
    self.lane_path_y_right = np.zeros((TRAJECTORY_SIZE,))
    self.lane_path_y_blend = np.zeros((TRAJECTORY_SIZE,))
    self.safe_idxs = np.zeros((TRAJECTORY_SIZE,), dtype=bool)

    self.params = Params()
    self.lane_width_estimate = FirstOrderFilter(float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1')), 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1'))
    self.spd_lane_width_spd = list(map(float, self.params.get("SpdLaneWidthSpd", encoding="utf8").split(',')))
    self.spd_lane_width_set = list(map(float, self.params.get("SpdLaneWidthSet", encoding="utf8").split(',')))

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

    self.left_curv_offset = int(self.params.get("LeftCurvOffsetAdj", encoding="utf8"))
    self.right_curv_offset = int(self.params.get("RightCurvOffsetAdj", encoding="utf8"))

    self.drive_close_to_edge = self.params.get_bool("CloseToRoadEdge")
    self.left_edge_offset = float(Decimal(self.params.get("LeftEdgeOffset", encoding="utf8")) * Decimal('0.01'))
    self.right_edge_offset = float(Decimal(self.params.get("RightEdgeOffset", encoding="utf8")) * Decimal('0.01'))

    self.speed_offset = self.params.get_bool("SpeedCameraOffset")

    self.road_edge_offset = 0.0

    self.lp_timer = 0
    self.lp_timer2 = 0
    self.lp_timer3 = 0
    
    self.total_camera_offset = self.camera_offset

  def parse_model(self, md, sm, v_ego):
    mode_select = int(sm['carState'].cruiseState.modeSel)
    current_road_offset = 0.0

    lean_offset = 0.15 if mode_select == 4 else 0

    if (self.left_curv_offset != 0 or self.right_curv_offset != 0) and v_ego > 8 and mode_select != 4:
      curvature = sm['controlsState'].curvature
      # right lane is minus
      lane_differ = round(self.lll_y[0] + self.rll_y[0], 2)
      if curvature > 0.0008 and self.left_curv_offset < 0 and lane_differ <= 0: # left curve
        if lane_differ > 0.6:
          lane_differ = 0.6          
        lean_offset = +round(abs(self.left_curv_offset) * abs(lane_differ * 0.05), 3) # move to left
      elif curvature > 0.0008 and self.left_curv_offset > 0 and lane_differ >= 0:
        if lane_differ > 0.6:
          lane_differ = 0.6
        lean_offset = -round(abs(self.left_curv_offset) * abs(lane_differ * 0.05), 3) # move to right
      elif curvature < -0.0008 and self.right_curv_offset < 0 and lane_differ <= 0: # right curve
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = +round(abs(self.right_curv_offset) * abs(lane_differ * 0.05), 3) # move to left
      elif curvature < -0.0008 and self.right_curv_offset > 0 and lane_differ >= 0:
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = -round(abs(self.right_curv_offset) * abs(lane_differ * 0.05), 3) # move to right
      else:
        lean_offset = 0

    self.lp_timer += DT_MDL
    if self.lp_timer > 1.0:
      self.lp_timer = 0.0
      self.speed_offset = self.params.get_bool("SpeedCameraOffset")
      if self.params.get_bool("OpkrLiveTunePanelEnable"):
        self.camera_offset = -(float(Decimal(self.params.get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))

    if self.drive_close_to_edge: # opkr
      left_edge_prob = min(max(1.0 - md.roadEdgeStds[0], 0.0), 1.0)
      left_nearside_prob = md.laneLineProbs[0]
      left_close_prob = md.laneLineProbs[1]
      right_close_prob = md.laneLineProbs[2]
      right_nearside_prob = md.laneLineProbs[3]
      right_edge_prob = min(max(1.0 - md.roadEdgeStds[1], 0.0), 1.0)

      self.lp_timer3 += DT_MDL
      if self.lp_timer3 > 3.0:
        self.lp_timer3 = 0.0
        if right_nearside_prob < 0.1 and left_nearside_prob < 0.1:
          self.road_edge_offset = 0.0
        elif right_edge_prob > 0.35 and right_nearside_prob < 0.2 and right_close_prob > 0.5 and left_nearside_prob >= right_nearside_prob:
          self.road_edge_offset = -self.right_edge_offset
        elif left_edge_prob > 0.35 and left_nearside_prob < 0.2 and left_close_prob > 0.5 and right_nearside_prob >= left_nearside_prob:
          self.road_edge_offset = -self.left_edge_offset
        else:
          self.road_edge_offset = 0.0
    else:
      self.road_edge_offset = 0.0
    if self.speed_offset:
      speed_offset = -interp(v_ego, SPEED_OFFSET_BP, SPEED_OFFSET_V)
    else:
      speed_offset = 0.0
    self.total_camera_offset = self.camera_offset + lean_offset + current_road_offset + self.road_edge_offset + speed_offset

    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      copy_to_f64(lane_lines[1].t, self.ll_t, TRAJECTORY_SIZE)
      copy_to_f64(lane_lines[2].t, self._ll_t_right, TRAJECTORY_SIZE)
      self.ll_t += self._ll_t_right
      self.ll_t *= 0.5
      # left and right ll x is the same
      copy_to_f64(lane_lines[1].x, self.ll_x, TRAJECTORY_SIZE)
      copy_to_f64(lane_lines[1].y, self.lll_y, TRAJECTORY_SIZE)
      if self.total_camera_offset != 0.0:
        self.lll_y += self.total_camera_offset
      copy_to_f64(lane_lines[2].y, self.rll_y, TRAJECTORY_SIZE)
      if self.total_camera_offset != 0.0:
        self.rll_y += self.total_camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    self.lp_timer2 += DT_MDL
    if self.lp_timer2 > 1.0:
      self.lp_timer2 = 0.0
      if self.params.get_bool("OpkrLiveTunePanelEnable"):
        self.path_offset = -(float(Decimal(self.params.get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    if self.path_offset != 0.0:
      path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    np.subtract(self.rll_y, self.lll_y, out=self.width_pts)
    width_at_t = interp(0.0, self.ll_x, self.width_pts)
    prob_mod_0 = interp(width_at_t, WIDTH_PROB_BP, WIDTH_PROB_V)
    width_at_t = interp(1.5 * (v_ego + 7), self.ll_x, self.width_pts)
    prob_mod_1 = interp(width_at_t, WIDTH_PROB_BP, WIDTH_PROB_V)
    width_at_t = interp(3.0 * (v_ego + 7), self.ll_x, self.width_pts)
    prob_mod_2 = interp(width_at_t, WIDTH_PROB_BP, WIDTH_PROB_V)
    mod = min(prob_mod_0, prob_mod_1, prob_mod_2)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, STD_PROB_BP, STD_PROB_V)
    r_std_mod = interp(self.rll_std, STD_PROB_BP, STD_PROB_V)
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, self.spd_lane_width_spd, self.spd_lane_width_set)
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    half_lane_width = clipped_lane_width / 2.0
    np.add(self.lll_y, half_lane_width, out=self.path_from_left_lane)
    np.subtract(self.rll_y, half_lane_width, out=self.path_from_right_lane)

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    np.multiply(self.path_from_left_lane, l_prob, out=self.lane_path_y)
    np.multiply(self.path_from_right_lane, r_prob, out=self.lane_path_y_right)
    self.lane_path_y += self.lane_path_y_right
    self.lane_path_y /= l_prob + r_prob + 0.0001
    np.isfinite(self.ll_t, out=self.safe_idxs)
    if self.safe_idxs[0]:
      if self.safe_idxs.all():
        interp_mono_to_f64(path_t, self.ll_t, self.lane_path_y, self.lane_path_y_interp, TRAJECTORY_SIZE)
        lane_path_y_interp = self.lane_path_y_interp
      else:
        lane_path_y_interp = np.interp(path_t, self.ll_t[self.safe_idxs], self.lane_path_y[self.safe_idxs])
      np.multiply(lane_path_y_interp, self.d_prob, out=self.lane_path_y_blend)
      path_y = path_xyz[:, 1]
      path_y *= (1.0 - self.d_prob)
      path_y += self.lane_path_y_blend
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz
