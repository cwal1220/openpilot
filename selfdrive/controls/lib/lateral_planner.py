import math
import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import copy_to_f64, interp, interp_to_f64
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MPC_COST_LAT, LAT_MPC_N
from selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE
from selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from common.conversions import Conversions as CV
from common.params import Params
from decimal import Decimal

LaneChangeState = log.LateralPlan.LaneChangeState
LATERAL_PLAN_SERVICES = ('carState', 'controlsState', 'modelV2')
CURVE_SPEED_START_BP = (10.0, 27.0)
CURVE_SPEED_START_V = (10.0, TRAJECTORY_SIZE - 10.0)
HEADING_COST_BP = (5.0, 10.0)
HEADING_COST_V = (MPC_COST_LAT.HEADING, 0.15)

def interp_mono_to_f64(x, xp, fp, out, count):
  for i in range(1, len(xp)):
    if xp[i] < xp[i - 1]:
      out[:count] = np.interp(x[:count], xp, fp)
      return
  interp_to_f64(x, xp, fp, out, count)

def is_mono_non_decreasing(xp):
  for i in range(1, len(xp)):
    if xp[i] < xp[i - 1]:
      return False
  return True

def gradient_nonuniform(out, values, x, dx1=None, dx2=None):
  if dx1 is None:
    dx1 = x[1:-1] - x[:-2]
  if dx2 is None:
    dx2 = x[2:] - x[1:-1]
  with np.errstate(divide='ignore', invalid='ignore'):
    out[1:-1] = (-dx2 / (dx1 * (dx1 + dx2))) * values[:-2] + \
                ((dx2 - dx1) / (dx1 * dx2)) * values[1:-1] + \
                (dx1 / (dx2 * (dx1 + dx2))) * values[2:]
    out[0] = (values[1] - values[0]) / (x[1] - x[0])
    out[-1] = (values[-1] - values[-2]) / (x[-1] - x[-2])

class LateralPlanner:
  def __init__(self, CP, use_lanelines=True, wide_camera=False):
    self.use_lanelines = use_lanelines
    self.LP = LanePlanner(wide_camera)
    self.DH = DesireHelper(CP)

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.path_xyz_stds = np.ones((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_curv_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE, dtype=np.float64)
    self.t_idxs_mpc = self.t_idxs[:LAT_MPC_N + 1]
    self.v_t_idxs = np.zeros(LAT_MPC_N + 1)
    self.y_pts = np.zeros(LAT_MPC_N + 1)
    self.heading_pts = np.zeros(LAT_MPC_N + 1)
    self.curv_rate_pts = np.zeros(LAT_MPC_N + 1)
    self.lat_mpc_p = np.zeros(2)
    self._curve_x = np.empty(TRAJECTORY_SIZE)
    self._curve_y = np.empty(TRAJECTORY_SIZE)
    self._curve_dy = np.empty(TRAJECTORY_SIZE)
    self._curve_d2y = np.empty(TRAJECTORY_SIZE)
    self._curve_dx1 = np.empty(TRAJECTORY_SIZE - 2)
    self._curve_dx2 = np.empty(TRAJECTORY_SIZE - 2)
    self._curve_curv = np.empty(TRAJECTORY_SIZE)
    self._curve_v_curvature = np.empty(TRAJECTORY_SIZE)
    self._d_path_lengths = np.empty(TRAJECTORY_SIZE)
    self._d_path_sq = np.empty((TRAJECTORY_SIZE, 3))
    self._lat_mpc_weight_key = None

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    self.params = Params()
    self.laneless_mode = int(self.params.get("LanelessMode", encoding="utf8"))
    self.laneless_mode_status = False
    self.laneless_mode_status_buffer = False

    self.standstill_elapsed_time = 0.0
    self.v_cruise_kph = 0
    self.stand_still = False
    
    self.second = 0.0
    self.model_speed = 255.0

    self.is_mph = not self.params.get_bool("IsMetric")

  def curve_speed(self, sm, v_ego, path_xyz_valid=False):
    md = sm['modelV2']
    curvature = sm['controlsState'].curvature
    if md is not None and len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
      if path_xyz_valid:
        curve_x = self.path_xyz[:, 0]
        curve_y = self.path_xyz[:, 1]
      else:
        copy_to_f64(md.position.x, self._curve_x, TRAJECTORY_SIZE)
        copy_to_f64(md.position.y, self._curve_y, TRAJECTORY_SIZE)
        curve_x = self._curve_x
        curve_y = self._curve_y
      np.subtract(curve_x[1:-1], curve_x[:-2], out=self._curve_dx1)
      np.subtract(curve_x[2:], curve_x[1:-1], out=self._curve_dx2)
      gradient_nonuniform(self._curve_dy, curve_y, curve_x, self._curve_dx1, self._curve_dx2)
      gradient_nonuniform(self._curve_d2y, self._curve_dy, curve_x, self._curve_dx1, self._curve_dx2)
      np.multiply(self._curve_dy, self._curve_dy, out=self._curve_curv)
      self._curve_curv += 1.0
      np.power(self._curve_curv, 1.5, out=self._curve_curv)
      np.divide(self._curve_d2y, self._curve_curv, out=self._curve_curv)
      start = int(interp(v_ego, CURVE_SPEED_START_BP, CURVE_SPEED_START_V)) # neokii's factor
      if abs(curvature) > 0.0008: # opkr
        curv = self._curve_curv[5:TRAJECTORY_SIZE-10]
      else:
        curv = self._curve_curv[start:min(start+10, TRAJECTORY_SIZE)]
      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = self._curve_v_curvature[:len(curv)]
      np.abs(curv, out=v_curvature)
      np.clip(v_curvature, 1e-4, None, out=v_curvature)
      np.divide(a_y_max, v_curvature, out=v_curvature)
      np.sqrt(v_curvature, out=v_curvature)
      model_speed = np.mean(v_curvature) * 0.9
      curve_speed = float(max(model_speed, 30 * CV.KPH_TO_MS))
      if np.isnan(curve_speed):
        curve_speed = 255
    else:
      curve_speed = 255
    return min(255, curve_speed * (CV.MS_TO_MPH if self.is_mph else CV.MS_TO_KPH))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self._lat_mpc_weight_key = None
    self.lat_mpc.reset(x0=self.x0)

  def set_lat_mpc_weights(self, path_weight, heading_weight, steer_rate_weight):
    weight_key = (path_weight, heading_weight, steer_rate_weight)
    if weight_key != self._lat_mpc_weight_key:
      self.lat_mpc.set_weights(path_weight, heading_weight, steer_rate_weight)
      self._lat_mpc_weight_key = weight_key

  def update(self, sm, CP):
    self.second += DT_MDL
    if self.second > 1.0:
      self.use_lanelines = not self.params.get_bool("EndToEndToggle")
      self.laneless_mode = int(self.params.get("LanelessMode", encoding="utf8"))
      self.second = 0.0

    self.v_cruise_kph = sm['controlsState'].vCruise
    self.stand_still = sm['carState'].standStill

  
    v_ego = sm['carState'].vEgo
    active = sm['controlsState'].active
    measured_curvature = sm['controlsState'].curvature

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md, sm, v_ego)
    path_xyz_valid = False
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      copy_to_f64(md.position.x, self.path_xyz[:, 0], TRAJECTORY_SIZE)
      copy_to_f64(md.position.y, self.path_xyz[:, 1], TRAJECTORY_SIZE)
      copy_to_f64(md.position.z, self.path_xyz[:, 2], TRAJECTORY_SIZE)
      copy_to_f64(md.position.t, self.t_idxs, TRAJECTORY_SIZE)
      copy_to_f64(md.orientation.z, self.plan_yaw, TRAJECTORY_SIZE)
      path_xyz_valid = True
    if len(md.position.xStd) == TRAJECTORY_SIZE:
      copy_to_f64(md.position.xStd, self.path_xyz_stds[:, 0], TRAJECTORY_SIZE)
      copy_to_f64(md.position.yStd, self.path_xyz_stds[:, 1], TRAJECTORY_SIZE)
      copy_to_f64(md.position.zStd, self.path_xyz_stds[:, 2], TRAJECTORY_SIZE)
    if sm.frame % 5 == 0:
      self.model_speed = self.curve_speed(sm, v_ego, path_xyz_valid)

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(CP, sm['carState'], sm['controlsState'], lane_change_prob, md)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    if self.use_lanelines:
      d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = False
    elif self.laneless_mode == 0:
      d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = False
    elif self.laneless_mode == 1:
      d_path_xyz = self.path_xyz
      # Heading cost is useful at low speed, otherwise end of plan can be off-heading
      heading_cost = interp(v_ego, HEADING_COST_BP, HEADING_COST_V)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, heading_cost, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = True
    elif self.laneless_mode == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 < 0.3) and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz = self.path_xyz
      heading_cost = interp(v_ego, HEADING_COST_BP, HEADING_COST_V)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, heading_cost, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = True
      self.laneless_mode_status_buffer = True
    elif self.laneless_mode == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 > 0.5) and \
      self.laneless_mode_status_buffer and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = False
      self.laneless_mode_status_buffer = False
    elif self.laneless_mode == 2 and self.laneless_mode_status_buffer == True and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz = self.path_xyz
      heading_cost = interp(v_ego, HEADING_COST_BP, HEADING_COST_V)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, heading_cost, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = True
    else:
      d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
      self.set_lat_mpc_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)
      self.laneless_mode_status = False
      self.laneless_mode_status_buffer = False

    np.multiply(d_path_xyz, d_path_xyz, out=self._d_path_sq)
    np.add(self._d_path_sq[:, 0], self._d_path_sq[:, 1], out=self._d_path_lengths)
    np.add(self._d_path_lengths, self._d_path_sq[:, 2], out=self._d_path_lengths)
    np.sqrt(self._d_path_lengths, out=self._d_path_lengths)
    d_path_lengths = self._d_path_lengths
    np.multiply(self.t_idxs_mpc, v_ego, out=self.v_t_idxs)
    if is_mono_non_decreasing(d_path_lengths):
      interp_to_f64(self.v_t_idxs, d_path_lengths, d_path_xyz[:, 1], self.y_pts, LAT_MPC_N + 1)
      interp_to_f64(self.v_t_idxs, d_path_lengths, self.plan_yaw, self.heading_pts, LAT_MPC_N + 1)
      interp_to_f64(self.v_t_idxs, d_path_lengths, self.plan_curv_rate, self.curv_rate_pts, LAT_MPC_N + 1)
    else:
      self.y_pts[:] = np.interp(self.v_t_idxs, d_path_lengths, d_path_xyz[:, 1])
      self.heading_pts[:] = np.interp(self.v_t_idxs, d_path_lengths, self.plan_yaw)
      self.curv_rate_pts[:] = np.interp(self.v_t_idxs, d_path_lengths, self.plan_curv_rate)

    assert len(self.y_pts) == LAT_MPC_N + 1
    assert len(self.heading_pts) == LAT_MPC_N + 1
    assert len(self.curv_rate_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * v_ego**2))
    self.lat_mpc_p[0] = v_ego
    self.lat_mpc_p[1] = lateral_factor
    self.lat_mpc.run(self.x0,
                     self.lat_mpc_p,
                     self.y_pts,
                     self.heading_pts,
                     self.curv_rate_pts)
    # init state for next
    # mpc.u_sol is the desired curvature rate given x0 curv state.
    # with x0[3] = measured_curvature, this would be the actual desired rate.
    # instead, interpolate x_sol so that x0[3] is the desired curvature for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs_mpc, self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 20000. or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=LATERAL_PLAN_SERVICES)

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = self.lat_mpc.x_sol[0:CONTROL_N, 3].tolist()
    lateralPlan.curvatureRates = self.lat_mpc.u_sol[:CONTROL_N - 1, 0].tolist() + [0.0]
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.modelSpeed = float(self.model_speed)
    lateralPlan.outputScale = float(self.DH.output_scale)
    lateralPlan.vCruiseSet = float(self.v_cruise_kph)
    lateralPlan.vCurvature = float(sm['controlsState'].curvature)
    lateralPlan.lanelessMode = bool(self.laneless_mode_status)
    lateralPlan.totalCameraOffset = float(self.LP.total_camera_offset)

    if self.stand_still:
      self.standstill_elapsed_time += DT_MDL
    else:
      self.standstill_elapsed_time = 0.0
    lateralPlan.standstillElapsedTime = int(self.standstill_elapsed_time)

    pm.send('lateralPlan', plan_send)
