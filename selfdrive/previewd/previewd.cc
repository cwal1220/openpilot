#include <linux/videodev2.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/k230_vvcam.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"

#include "display.h"
#include "v4l2-drm.h"

ExitHandler do_exit;

namespace {

constexpr unsigned K230_SENSOR_WIDTH = 1920;
constexpr unsigned K230_SENSOR_HEIGHT = 1080;
constexpr int K230_DEFAULT_V4L2_BUFFERS = 5;
constexpr int K230_OVERLAY_VERTICES = TRAJECTORY_SIZE * 2;
constexpr int K230_PREVIEW_HUD_COMPACT = 0;
constexpr int K230_PREVIEW_HUD_FULL = 1;
constexpr int K230_PREVIEW_HUD_MINIMAL = 2;
constexpr uint64_t K230_PREVIEW_PARAM_CHECK_FRAMES = 30;

constexpr float K230_OV5647_FX = 1625.7416788144435f;
constexpr float K230_OV5647_FY = 1585.9830269782024f;
constexpr float K230_OV5647_CX = 946.13450988811394f;
constexpr float K230_OV5647_CY = 537.34063862123787f;
constexpr int K230_PREVIEW_RETRY_MS = 300;
constexpr uint32_t K230_PREVIEW_TIMEOUTS_BEFORE_RESTART = 3;
constexpr float K230_MODEL_MIN_DEPTH = 0.05f;
constexpr float K230_MODEL_TOP_GUARD_PX = 96.0f;

struct K230PreviewConfig {
  int device = -1;
  unsigned width = 0;
  unsigned height = 0;
  int v4l2_buffers = K230_DEFAULT_V4L2_BUFFERS;
};

struct K230Point {
  float x = 0.0f;
  float y = 0.0f;
};

struct K230Poly {
  std::array<K230Point, K230_OVERLAY_VERTICES> points = {};
  int cnt = 0;
};

struct K230Rect {
  int x0 = 0;
  int y0 = 0;
  int x1 = -1;
  int y1 = -1;
};

struct K230Mat3 {
  float v[9] = {};
};

struct K230HudState {
  bool enabled = false;
  bool engageable = false;
  bool active = false;
  bool control_allowed = false;
  bool brake_pressed = false;
  bool brake_lights = false;
  bool gas_pressed = false;
  bool left_blinker = false;
  bool right_blinker = false;
  bool left_blindspot = false;
  bool right_blindspot = false;
  bool standstill = false;
  bool brake_hold = false;
  bool cruise_acc = false;
  bool driver_acc = false;
  bool laneless = false;
  bool lead_status = false;
  bool lead2_status = false;
  bool longitudinal_control = false;
  bool long_plan_seen = false;
  bool long_plan_alive = false;
  bool fcw = false;
  bool model_seen = false;
  bool model_alive = false;
  bool controls_seen = false;
  bool controls_alive = false;
  bool car_seen = false;
  bool car_alive = false;
  bool panda_seen = false;
  bool panda_alive = false;
  bool ignition = false;
  bool plan_seen = false;
  bool plan_alive = false;
  bool calib_seen = false;
  bool calib_alive = false;
  bool steering_pressed = false;
  bool steer_warning = false;
  bool speed_limit_pause = false;
  bool gap_by_speed = false;
  bool dm_active = false;

  float speed_kph = 0.0f;
  float v_cruise = 0.0f;
  float safety_speed = 0.0f;
  float v_set_dis = 0.0f;
  float steering_angle = 0.0f;
  float desired_angle = 0.0f;
  float accel = 0.0f;
  float a_req = 0.0f;
  float output_scale = 0.0f;
  float steer_cmd = 0.0f;
  float lane_width = 0.0f;
  float d_prob = 0.0f;
  float l_prob = 0.0f;
  float r_prob = 0.0f;
  float steer_ratio = 0.0f;
  float angle_offset_avg = 0.0f;
  float stiffness_factor = 0.0f;
  float cpu_usage = 0.0f;
  float cpu_temp = 0.0f;
  float memory_usage = 0.0f;
  float free_space_percent = 0.0f;
  float lead_d = 0.0f;
  float lead_v = 0.0f;
  float lead_y = 0.0f;
  float lead2_d = 0.0f;
  float lead2_v = 0.0f;
  float lead2_y = 0.0f;
  float radar_distance = 0.0f;
  float steering_torque = 0.0f;
  float engine_rpm = 0.0f;
  float charge_meter = 0.0f;
  float steer_actuator_delay = 0.0f;
  float gps_accuracy = 0.0f;
  float altitude = 0.0f;
  float bearing_deg = 0.0f;
  float model_execution_ms = 0.0f;
  float total_camera_offset = 0.0f;
  float standstill_elapsed = 0.0f;
  float calib_pitch_deg = 0.0f;
  float calib_yaw_deg = 0.0f;
  float limit_speed_camera = 0.0f;
  float limit_speed_camera_dist = 0.0f;
  float dynamic_tr_value = 0.0f;
  float stopline_prob = 0.0f;
  float stopline_distance = 0.0f;
  std::array<float, 4> lane_line_probs = {};
  std::array<float, 2> road_edge_confs = {};
  std::array<float, 4> tpms = {};

  uint8_t cruise_gap = 0;
  uint8_t lateral_control_method = 0;
  uint8_t dynamic_tr_mode = 0;
  uint8_t longitudinal_source = 0;
  int tpms_unit = 0;
  int gear_step = 0;
  int satellite_count = 0;
  int alert_size = 0;
  int alert_status = 0;
  int gear = 0;
  int calib_status = 0;
  int calib_percent = 0;
  int battery_percent = 0;
  int fan_speed = 0;
  int thermal_status = 0;
  int network_type = 0;
  int network_strength = 0;
  int map_sign = 0;
  int map_sign_cam = 0;
  std::string alert_text1;
  std::string alert_text2;
  std::string alert_type;
  std::string alert_msg1;
  std::string alert_msg2;
  std::string alert_msg3;
  std::string battery_status;
};

uint32_t argb(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint32_t>(a) << 24) |
         (static_cast<uint32_t>(r) << 16) |
         (static_cast<uint32_t>(g) << 8) |
         static_cast<uint32_t>(b);
}

K230Mat3 matmul3(const K230Mat3 &a, const K230Mat3 &b) {
  K230Mat3 out = {};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.v[r * 3 + c] = a.v[r * 3 + 0] * b.v[0 * 3 + c] +
                         a.v[r * 3 + 1] * b.v[1 * 3 + c] +
                         a.v[r * 3 + 2] * b.v[2 * 3 + c];
    }
  }
  return out;
}

K230Mat3 view_from_calib_from_rpy(float roll, float pitch, float yaw) {
  const float cr = std::cos(roll), sr = std::sin(roll);
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  const float cy = std::cos(yaw), sy = std::sin(yaw);

  const K230Mat3 rot_x = {{1, 0, 0,
                           0, cr, -sr,
                           0, sr, cr}};
  const K230Mat3 rot_y = {{cp, 0, sp,
                           0, 1, 0,
                           -sp, 0, cp}};
  const K230Mat3 rot_z = {{cy, -sy, 0,
                           sy, cy, 0,
                           0, 0, 1}};
  const K230Mat3 view_from_device = {{0, 1, 0,
                                      0, 0, 1,
                                      1, 0, 0}};
  return matmul3(view_from_device, matmul3(rot_z, matmul3(rot_y, rot_x)));
}

K230PreviewConfig read_config() {
  K230PreviewConfig cfg = {};
  if (!k230_vvcam::wait_for_ready()) {
    throw std::runtime_error("K230 vvcam daemon/video nodes not ready");
  }
  const int video00 = k230_vvcam::detect_vvcam_video00();
  if (video00 < 0) {
    throw std::runtime_error("K230 vvcam-video.0.0 not found");
  }
  cfg.device = video00;
  return cfg;
}

class K230PreviewRuntime {
public:
  K230PreviewRuntime(const K230PreviewConfig &init_cfg, struct display *init_display)
      : cfg(init_cfg), display(init_display),
        sm({"modelV2", "liveCalibration", "controlsState", "carState", "deviceState",
            "lateralPlan", "liveParameters", "radarState", "pandaStates", "carParams",
            "ubloxGnss", "gpsLocationExternal", "driverMonitoringState", "longitudinalPlan"}) {
    view_from_calib = view_from_calib_from_rpy(0.0f, 0.0f, 0.0f);
  }

  int run() {
    setup_preview_capture();
    if (do_exit) return 0;

    setup_video_buffers();
    setup_overlay();

    if (display_update_buffer(video_buffers[0], 0, 0) != 0) {
      LOGE("K230 previewd initial video update failed");
    }
    if (overlay_buffer != nullptr && display_update_buffer(overlay_buffer, 0, 0) != 0) {
      LOGE("K230 previewd initial overlay update failed");
    }
    if (display_commit(display) == 0) {
      display_wait_vsync(display);
    }

    uint32_t capture_timeouts = 0;
    while (!do_exit) {
      if (v4l2_drm_dump(&ctx, 1000) != 0) {
        const int err = errno;
        if (err == EINTR) {
          continue;
        }
        if (err == EAGAIN || err == ETIMEDOUT) {
          ++capture_timeouts;
          if (capture_timeouts == 1 || capture_timeouts >= K230_PREVIEW_TIMEOUTS_BEFORE_RESTART) {
            LOGE("K230 previewd capture timeout on /dev/video%d, consecutive=%u", cfg.device, capture_timeouts);
          }
          if (capture_timeouts >= K230_PREVIEW_TIMEOUTS_BEFORE_RESTART) {
            if (!restart_preview_capture(err)) {
              break;
            }
            capture_timeouts = 0;
          }
          continue;
        }
        LOGE("K230 previewd capture failed errno=%d (%s), restarting /dev/video%d",
             err, std::strerror(err), cfg.device);
        if (!restart_preview_capture(err)) {
          break;
        }
        capture_timeouts = 0;
        continue;
      }
      capture_timeouts = 0;

      auto *src = static_cast<const uint8_t *>(ctx.buffers[ctx.vbuffer.index].mmap);
      video_buffer_index = (video_buffer_index + 1) % video_buffers.size();
      copy_nv12(src, video_buffers[video_buffer_index]);
      v4l2_drm_dump_release(&ctx);

      update_overlay();
      if (display_update_buffer(video_buffers[video_buffer_index], 0, 0) != 0) {
        LOGE("K230 previewd video update failed");
      }
      if (display_commit(display) == 0) {
        display_wait_vsync(display);
      } else {
        LOGE("K230 previewd display commit failed");
      }
    }
    return 0;
  }

  ~K230PreviewRuntime() {
    if (setup) {
      v4l2_drm_stop(&ctx);
      setup = false;
    }
    for (display_buffer *buffer : overlay_buffers) {
      display_free_buffer(buffer);
    }
    overlay_buffers.clear();
    overlay_buffer = nullptr;
    if (overlay_plane != nullptr) {
      display_free_plane(overlay_plane);
      overlay_plane = nullptr;
    }
    for (display_buffer *buffer : video_buffers) {
      display_free_buffer(buffer);
    }
    video_buffers.clear();
    if (video_plane != nullptr) {
      display_free_plane(video_plane);
      video_plane = nullptr;
    }
    if (display != nullptr) {
      display_exit(display);
      display = nullptr;
    }
  }

private:
  void setup_preview_capture() {
    v4l2_drm_default_context(&ctx);
    ctx.device = cfg.device;
    ctx.width = cfg.width;
    ctx.height = cfg.height;
    ctx.video_format = V4L2_PIX_FMT_NV12;
    ctx.buffer_num = static_cast<unsigned>(cfg.v4l2_buffers);

    LOGW("K230 previewd opening /dev/video%d %ux%u NV12 drm-rotate-to %ux%u",
         cfg.device, cfg.width, cfg.height, display->width, display->height);

    {
      k230_vvcam::SetupLock setup_lock;
      if (!setup_lock.locked()) {
        LOGW("K230 previewd could not lock vvcam setup");
      }

      if (v4l2_drm_setup(&ctx, 1) != 0) {
        throw std::runtime_error(util::string_format("v4l2_drm_setup preview capture failed for /dev/video%d errno=%d (%s)",
                                                     cfg.device, errno, std::strerror(errno)));
      }
    }
    setup = true;
    if (v4l2_drm_start(&ctx) != 0) {
      const int start_errno = errno;
      v4l2_drm_stop(&ctx);
      setup = false;
      errno = start_errno;
      throw std::runtime_error(util::string_format("v4l2_drm_start preview capture failed for /dev/video%d errno=%d (%s)",
                                                   cfg.device, errno, std::strerror(errno)));
    }
  }

  bool restart_preview_capture(int err) {
    LOGE("K230 previewd restarting /dev/video%d capture after errno=%d (%s)",
         cfg.device, err, std::strerror(err));
    if (setup) {
      v4l2_drm_stop(&ctx);
      setup = false;
    }
    util::sleep_for(K230_PREVIEW_RETRY_MS);

    if (do_exit) return false;
    setup_preview_capture();
    return true;
  }

  void setup_video_buffers() {
    video_plane = display_get_plane(display, DRM_FORMAT_NV12);
    if (video_plane == nullptr) {
      throw std::runtime_error("K230 previewd could not allocate NV12 video plane");
    }
    video_plane->drm_rotation = rotation_90;

    for (int i = 0; i < 3; ++i) {
      display_buffer *buffer = display_allocate_buffer(video_plane, cfg.width, cfg.height);
      if (buffer == nullptr) {
        throw std::runtime_error("K230 previewd could not allocate NV12 display buffer");
      }
      const size_t y_size = static_cast<size_t>(buffer->stride) * buffer->height;
      std::memset(buffer->map, 16, y_size);
      std::memset(static_cast<uint8_t *>(buffer->map) + y_size, 128, buffer->size - y_size);
      video_buffers.push_back(buffer);
    }
  }

  void copy_nv12(const uint8_t *src, display_buffer *dst) {
    const int src_w = static_cast<int>(cfg.width);
    const int src_h = static_cast<int>(cfg.height);
    auto *dst_y = static_cast<uint8_t *>(dst->map);
    auto *dst_uv = dst_y + dst->stride * dst->height;
    const uint8_t *src_y = src;
    const uint8_t *src_uv = src + src_w * src_h;

    for (int y = 0; y < src_h; ++y) {
      std::memcpy(dst_y + static_cast<size_t>(y) * dst->stride,
                  src_y + static_cast<size_t>(y) * src_w,
                  src_w);
    }
    for (int y = 0; y < src_h / 2; ++y) {
      std::memcpy(dst_uv + static_cast<size_t>(y) * dst->stride,
                  src_uv + static_cast<size_t>(y) * src_w,
                  src_w);
    }
  }

  void setup_overlay() {
    overlay_plane = display_get_plane(display, DRM_FORMAT_ARGB8888);
    if (overlay_plane == nullptr) {
      LOGE("K230 previewd could not allocate ARGB overlay plane");
      return;
    }
    overlay_plane->drm_rotation = rotation_90;

    for (int i = 0; i < 2; ++i) {
      display_buffer *buffer = display_allocate_buffer(overlay_plane, cfg.width, cfg.height);
      if (buffer == nullptr) {
        LOGE("K230 previewd could not allocate ARGB overlay buffer");
        for (display_buffer *allocated : overlay_buffers) {
          display_free_buffer(allocated);
        }
        overlay_buffers.clear();
        display_free_plane(overlay_plane);
        overlay_plane = nullptr;
        return;
      }
      std::memset(buffer->map, 0, buffer->size);
      overlay_buffers.push_back(buffer);
    }
    overlay_dirty_rects.resize(overlay_buffers.size());
    overlay_buffer = overlay_buffers[0];
  }

  float finite_or_zero(float value) const {
    return std::isfinite(value) ? value : 0.0f;
  }

  template <typename List>
  float average_list(const List &values) const {
    float sum = 0.0f;
    int count = 0;
    for (auto value : values) {
      const float v = static_cast<float>(value);
      if (std::isfinite(v)) {
        sum += v;
        ++count;
      }
    }
    return count > 0 ? sum / count : 0.0f;
  }

  float lateral_output_from_controls(const cereal::ControlsState::Reader &cs) const {
    const auto state = cs.getLateralControlState();
    switch (cs.getLateralControlMethod()) {
      case 0: return finite_or_zero(state.getPidState().getOutput());
      case 1: return finite_or_zero(state.getIndiState().getOutput());
      case 2: return finite_or_zero(state.getLqrState().getOutput());
      case 3: return finite_or_zero(state.getTorqueState().getOutput());
      case 4: return finite_or_zero(state.getAtomState().getOutput());
      default: return 0.0f;
    }
  }

  std::string sanitize_text(const char *text, size_t max_len) const {
    std::string out;
    if (text == nullptr) return out;

    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(text);
         *p != '\0' && out.size() < max_len; ++p) {
      char c = static_cast<char>(*p);
      if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
      if (c >= 32 && c <= 126) {
        out.push_back(c);
      } else if (!out.empty() && out.back() != ' ') {
        out.push_back(' ');
      }
    }
    return out;
  }

  bool update_state() {
    bool changed = false;
    sm.update(0);

    auto mark_bool = [&changed](bool &field, bool value) {
      if (field != value) {
        field = value;
        changed = true;
      }
    };
    mark_bool(hud.model_seen, sm.rcv_frame("modelV2") > 0);
    mark_bool(hud.model_alive, sm.alive("modelV2"));
    mark_bool(hud.controls_seen, sm.rcv_frame("controlsState") > 0);
    mark_bool(hud.controls_alive, sm.alive("controlsState"));
    mark_bool(hud.car_seen, sm.rcv_frame("carState") > 0);
    mark_bool(hud.car_alive, sm.alive("carState"));
    mark_bool(hud.panda_seen, sm.rcv_frame("pandaStates") > 0);
    mark_bool(hud.panda_alive, sm.alive("pandaStates"));
    mark_bool(hud.plan_seen, sm.rcv_frame("lateralPlan") > 0);
    mark_bool(hud.plan_alive, sm.alive("lateralPlan"));
    mark_bool(hud.long_plan_seen, sm.rcv_frame("longitudinalPlan") > 0);
    mark_bool(hud.long_plan_alive, sm.alive("longitudinalPlan"));
    mark_bool(hud.calib_seen, sm.rcv_frame("liveCalibration") > 0);
    mark_bool(hud.calib_alive, sm.alive("liveCalibration"));

    if (sm.updated("liveCalibration")) {
      auto calib = sm["liveCalibration"].getLiveCalibration();
      auto rpy = calib.getRpyCalib();
      if (rpy.size() >= 3 && std::isfinite(rpy[0]) && std::isfinite(rpy[1]) && std::isfinite(rpy[2])) {
        view_from_calib = view_from_calib_from_rpy(rpy[0], rpy[1], rpy[2]);
        hud.calib_pitch_deg = rpy[1] * (180.0f / 3.14159265358979323846f);
        hud.calib_yaw_deg = rpy[2] * (180.0f / 3.14159265358979323846f);
        changed = true;
      }
      hud.calib_status = calib.getCalStatus();
      hud.calib_percent = calib.getCalPerc();
      changed = true;
    }

    if (sm.updated("controlsState")) {
      auto cs = sm["controlsState"].getControlsState();
      hud.enabled = cs.getEnabled();
      hud.active = cs.getActive();
      hud.engageable = cs.getEngageable();
      hud.lateral_control_method = cs.getLateralControlMethod();
      hud.v_cruise = finite_or_zero(cs.getVCruise());
      hud.safety_speed = finite_or_zero(cs.getSafetySpeed());
      hud.desired_angle = finite_or_zero(cs.getSteeringAngleDesiredDeg());
      hud.accel = finite_or_zero(cs.getAccel());
      hud.steer_cmd = finite_or_zero(cs.getSteer());
      hud.output_scale = lateral_output_from_controls(cs);
      hud.limit_speed_camera = finite_or_zero(cs.getLimitSpeedCamera());
      hud.limit_speed_camera_dist = finite_or_zero(cs.getLimitSpeedCameraDist());
      hud.map_sign = cs.getMapSign();
      hud.map_sign_cam = cs.getMapSignCam();
      hud.dynamic_tr_mode = cs.getDynamicTRMode();
      hud.dynamic_tr_value = finite_or_zero(cs.getDynamicTRValue());
      hud.speed_limit_pause = cs.getPauseSpdLimit();
      hud.gap_by_speed = cs.getGapBySpeedOn();
      hud.alert_size = static_cast<int>(cs.getAlertSize());
      hud.alert_status = static_cast<int>(cs.getAlertStatus());
      hud.alert_text1 = sanitize_text(cs.getAlertText1().cStr(), 32);
      hud.alert_text2 = sanitize_text(cs.getAlertText2().cStr(), 32);
      hud.alert_type = sanitize_text(cs.getAlertType().cStr(), 24);
      hud.alert_msg1 = sanitize_text(cs.getAlertTextMsg1().cStr(), 28);
      hud.alert_msg2 = sanitize_text(cs.getAlertTextMsg2().cStr(), 28);
      hud.alert_msg3 = sanitize_text(cs.getAlertTextMsg3().cStr(), 28);
      changed = true;
    }

    if (sm.updated("carState")) {
      auto car = sm["carState"].getCarState();
      hud.speed_kph = std::max(0.0f, finite_or_zero(car.getVEgo() * 3.6f));
      hud.v_set_dis = finite_or_zero(car.getVSetDis());
      hud.cruise_gap = car.getCruiseGapSet();
      hud.steering_angle = finite_or_zero(car.getSteeringAngleDeg());
      hud.steering_pressed = car.getSteeringPressed();
      hud.steer_warning = car.getSteerFaultTemporary();
      hud.brake_pressed = car.getBrakePressed();
      hud.brake_lights = car.getBrakeLights();
      hud.gas_pressed = car.getGasPressed();
      hud.left_blinker = car.getLeftBlinker();
      hud.right_blinker = car.getRightBlinker();
      hud.left_blindspot = car.getLeftBlindspot();
      hud.right_blindspot = car.getRightBlindspot();
      hud.radar_distance = finite_or_zero(car.getRadarDistance());
      hud.standstill = car.getStandstill() || car.getStandStill();
      hud.brake_hold = car.getBrakeHold() || car.getBrakeHoldActive();
      hud.cruise_acc = car.getCruiseAccStatus();
      hud.driver_acc = car.getDriverAcc();
      hud.a_req = finite_or_zero(car.getAReqValue());
      hud.gear = static_cast<int>(car.getGearShifter());
      hud.steering_torque = finite_or_zero(car.getSteeringTorque());
      hud.engine_rpm = finite_or_zero(car.getEngineRpm());
      hud.gear_step = car.getGearStep();
      hud.charge_meter = finite_or_zero(car.getChargeMeter());
      hud.tpms_unit = car.getTpms().getUnit();
      hud.tpms[0] = finite_or_zero(car.getTpms().getFl());
      hud.tpms[1] = finite_or_zero(car.getTpms().getFr());
      hud.tpms[2] = finite_or_zero(car.getTpms().getRl());
      hud.tpms[3] = finite_or_zero(car.getTpms().getRr());
      changed = true;
    }

    if (sm.updated("deviceState")) {
      auto ds = sm["deviceState"].getDeviceState();
      hud.cpu_usage = average_list(ds.getCpuUsagePercent());
      hud.cpu_temp = average_list(ds.getCpuTempC());
      hud.memory_usage = static_cast<float>(ds.getMemoryUsagePercent());
      hud.free_space_percent = finite_or_zero(ds.getFreeSpacePercent());
      hud.battery_percent = ds.getBatteryPercent();
      hud.fan_speed = ds.getFanSpeedPercentDesired();
      hud.thermal_status = static_cast<int>(ds.getThermalStatus());
      hud.network_type = static_cast<int>(ds.getNetworkType());
      hud.network_strength = static_cast<int>(ds.getNetworkStrength());
      hud.battery_status = sanitize_text(ds.getBatteryStatus().cStr(), 12);
      changed = true;
    }

    if (sm.updated("lateralPlan")) {
      auto lp = sm["lateralPlan"].getLateralPlan();
      hud.lane_width = finite_or_zero(lp.getLaneWidth());
      hud.d_prob = finite_or_zero(lp.getDProb());
      hud.l_prob = finite_or_zero(lp.getLProb());
      hud.r_prob = finite_or_zero(lp.getRProb());
      if (!hud.controls_seen) {
        hud.output_scale = finite_or_zero(lp.getOutputScale());
      }
      hud.standstill_elapsed = finite_or_zero(lp.getStandstillElapsedTime());
      hud.total_camera_offset = finite_or_zero(lp.getTotalCameraOffset());
      hud.laneless = lp.getLanelessMode();
      changed = true;
    }

    if (sm.updated("liveParameters")) {
      auto lp = sm["liveParameters"].getLiveParameters();
      hud.steer_ratio = finite_or_zero(lp.getSteerRatio());
      hud.angle_offset_avg = finite_or_zero(lp.getAngleOffsetAverageDeg());
      hud.stiffness_factor = finite_or_zero(lp.getStiffnessFactor());
      changed = true;
    }

    if (sm.updated("radarState")) {
      auto radar = sm["radarState"].getRadarState();
      auto lead = radar.getLeadOne();
      hud.lead_status = lead.getStatus();
      hud.lead_d = finite_or_zero(lead.getDRel());
      hud.lead_v = finite_or_zero(lead.getVRel());
      hud.lead_y = finite_or_zero(lead.getYRel());
      auto lead2 = radar.getLeadTwo();
      hud.lead2_status = lead2.getStatus();
      hud.lead2_d = finite_or_zero(lead2.getDRel());
      hud.lead2_v = finite_or_zero(lead2.getVRel());
      hud.lead2_y = finite_or_zero(lead2.getYRel());
      changed = true;
    }

    if (sm.updated("longitudinalPlan")) {
      auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
      hud.fcw = lp.getFcw();
      hud.longitudinal_source = static_cast<uint8_t>(lp.getLongitudinalPlanSource());
      hud.stopline_prob = finite_or_zero(lp.getStoplineProb());
      const auto stopline = lp.getStopLine();
      hud.stopline_distance = stopline.size() > 0 ? finite_or_zero(static_cast<float>(stopline[std::min(12, static_cast<int>(stopline.size()) - 1)])) : 0.0f;
      if (lp.getDynamicTRMode() != 0 || lp.getDynamicTRValue() > 0.0f) {
        hud.dynamic_tr_mode = lp.getDynamicTRMode();
        hud.dynamic_tr_value = finite_or_zero(lp.getDynamicTRValue());
      }
      changed = true;
    }

    if (sm.updated("modelV2")) {
      auto model = sm["modelV2"].getModelV2();
      hud.model_execution_ms = finite_or_zero(model.getModelExecutionTime() * 1000.0f);

      const auto lane_probs = model.getLaneLineProbs();
      for (int i = 0; i < 4 && i < static_cast<int>(lane_probs.size()); ++i) {
        hud.lane_line_probs[i] = finite_or_zero(lane_probs[i]);
      }

      const auto road_edge_stds = model.getRoadEdgeStds();
      for (int i = 0; i < 2 && i < static_cast<int>(road_edge_stds.size()); ++i) {
        hud.road_edge_confs[i] = std::clamp(1.0f - finite_or_zero(road_edge_stds[i]), 0.0f, 1.0f);
      }
      changed = true;
    }

    if (sm.updated("pandaStates")) {
      auto panda_states = sm["pandaStates"].getPandaStates();
      hud.control_allowed = false;
      hud.ignition = false;
      for (const auto &panda_state : panda_states) {
        hud.control_allowed = hud.control_allowed || panda_state.getControlsAllowed();
        hud.ignition = hud.ignition || panda_state.getIgnitionLine() || panda_state.getIgnitionCan();
      }
      changed = true;
    }

    if (sm.updated("carParams")) {
      auto cp = sm["carParams"].getCarParams();
      hud.longitudinal_control = cp.getOpenpilotLongitudinalControl();
      hud.steer_actuator_delay = finite_or_zero(cp.getSteerActuatorDelay());
      changed = true;
    }

    if (sm.updated("ubloxGnss")) {
      auto ublox = sm["ubloxGnss"].getUbloxGnss();
      if (ublox.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
        hud.satellite_count = ublox.getMeasurementReport().getNumMeas();
        changed = true;
      }
    }

    if (sm.updated("gpsLocationExternal")) {
      auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
      hud.gps_accuracy = finite_or_zero(gps.getAccuracy());
      hud.altitude = finite_or_zero(static_cast<float>(gps.getAltitude()));
      hud.bearing_deg = finite_or_zero(gps.getBearingDeg());
      changed = true;
    }

    if (sm.updated("driverMonitoringState")) {
      hud.dm_active = sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode();
      changed = true;
    }

    return changed;
  }

  bool update_params() {
    if (!first_overlay && overlay_frame_counter - last_param_check_frame < K230_PREVIEW_PARAM_CHECK_FRAMES) {
      return false;
    }
    last_param_check_frame = overlay_frame_counter;

    const std::string value = params.get("K230PreviewHudMode");
    int next_mode = K230_PREVIEW_HUD_FULL;
    if (!value.empty()) {
      next_mode = std::atoi(value.c_str());
    }
    next_mode = std::clamp(next_mode, K230_PREVIEW_HUD_COMPACT, K230_PREVIEW_HUD_MINIMAL);

    const std::string metric_value = params.get("IsMetric");
    const bool next_metric = metric_value.empty() ? true : metric_value == "1";
    const bool next_show_stop_line = params.getBool("ShowStopLine");
    const bool changed = next_mode != hud_mode || next_metric != is_metric || next_show_stop_line != show_stop_line;
    hud_mode = next_mode;
    is_metric = next_metric;
    show_stop_line = next_show_stop_line;
    return changed;
  }

  void update_overlay() {
    const bool state_changed = update_state();
    const bool params_changed = update_params();
    if (overlay_buffers.empty()) return;

    const uint64_t model_frame = sm.rcv_frame("modelV2");
    ++overlay_frame_counter;
    pending_hud_update = pending_hud_update || state_changed || params_changed;
    const bool hud_redraw = pending_hud_update && (first_overlay || overlay_frame_counter % 3 == 0);
    if (!first_overlay && !hud_redraw && model_frame == last_model_frame) {
      return;
    }

    overlay_buffer_index = (overlay_buffer_index + 1) % overlay_buffers.size();
    overlay_buffer = overlay_buffers[overlay_buffer_index];
    clear_overlay_rects(overlay_buffer, overlay_dirty_rects[overlay_buffer_index]);
    current_overlay_dirty_rects.clear();
    if (model_frame > 0 && !draw_model(sm["modelV2"].getModelV2())) {
      clear_overlay_buffer(overlay_buffer);
      current_overlay_dirty_rects.clear();
    }
    draw_hud();
    overlay_dirty_rects[overlay_buffer_index] = current_overlay_dirty_rects;
    pending_hud_update = false;
    first_overlay = false;
    last_model_frame = model_frame;

    if (display_update_buffer(overlay_buffer, 0, 0) != 0) {
      LOGE("K230 previewd overlay update failed");
    }
  }

  bool rect_valid(const K230Rect &rect) const {
    return rect.x0 <= rect.x1 && rect.y0 <= rect.y1;
  }

  void clear_overlay_rect(display_buffer *buffer, const K230Rect &rect) {
    if (!rect_valid(rect)) return;

    auto *base = static_cast<uint8_t *>(buffer->map);
    const int x0 = std::max(0, rect.x0);
    const int y0 = std::max(0, rect.y0);
    const int x1 = std::min(static_cast<int>(buffer->width) - 1, rect.x1);
    const int y1 = std::min(static_cast<int>(buffer->height) - 1, rect.y1);
    if (x0 > x1 || y0 > y1) return;

    const size_t bytes = static_cast<size_t>(x1 - x0 + 1) * sizeof(uint32_t);
    for (int y = y0; y <= y1; ++y) {
      std::memset(base + static_cast<size_t>(y) * buffer->stride + static_cast<size_t>(x0) * sizeof(uint32_t), 0, bytes);
    }
  }

  void clear_overlay_rects(display_buffer *buffer, const std::vector<K230Rect> &rects) {
    for (const K230Rect &rect : rects) {
      clear_overlay_rect(buffer, rect);
    }
  }

  void clear_overlay_buffer(display_buffer *buffer) {
    if (buffer == nullptr) return;
    std::memset(buffer->map, 0, static_cast<size_t>(buffer->stride) * buffer->height);
  }

  void include_overlay_rect(const K230Rect &rect) {
    if (!rect_valid(rect)) return;
    add_merged_rect(current_overlay_dirty_rects, rect);
  }

  int path_length_idx(const cereal::ModelDataV2::XYZTData::Reader &line, float path_height) const {
    const auto xs = line.getX();
    int max_idx = 0;
    for (int i = 0; i < TRAJECTORY_SIZE && xs[i] < path_height; ++i) {
      max_idx = i;
    }
    return max_idx;
  }

  bool project(float in_x, float in_y, float in_z, K230Point *out) const {
    const float sx = static_cast<float>(cfg.width) / K230_SENSOR_WIDTH;
    const float sy = static_cast<float>(cfg.height) / K230_SENSOR_HEIGHT;
    const float fx = K230_OV5647_FX * sx;
    const float fy = K230_OV5647_FY * sy;
    const float cx = K230_OV5647_CX * sx;
    const float cy = K230_OV5647_CY * sy;

    const float ex = view_from_calib.v[0] * in_x + view_from_calib.v[1] * in_y + view_from_calib.v[2] * in_z;
    const float ey = view_from_calib.v[3] * in_x + view_from_calib.v[4] * in_y + view_from_calib.v[5] * in_z;
    const float ez = view_from_calib.v[6] * in_x + view_from_calib.v[7] * in_y + view_from_calib.v[8] * in_z;
    if (!std::isfinite(ez) || ez <= K230_MODEL_MIN_DEPTH) return false;

    const float px = (fx * ex + cx * ez) / ez;
    const float py = (fy * ey + cy * ez) / ez;
    if (!std::isfinite(px) || !std::isfinite(py) ||
        px < -200.0f || px > cfg.width + 200.0f ||
        py < -200.0f || py > cfg.height + 200.0f) {
      return false;
    }

    *out = {px, py};
    return out->x >= -200.0f && out->x <= overlay_buffer->width + 200.0f &&
           out->y >= -200.0f && out->y <= overlay_buffer->height + 200.0f;
  }

  void build_line_poly(const cereal::ModelDataV2::XYZTData::Reader &line, float y_off, float z_off,
                       int max_idx, K230Poly *poly) const {
    poly->cnt = 0;
    const auto xs = line.getX();
    const auto ys = line.getY();
    const auto zs = line.getZ();
    for (int i = 0; i <= max_idx && poly->cnt < K230_OVERLAY_VERTICES; ++i) {
      K230Point point;
      if (project(xs[i], ys[i] - y_off, zs[i] + z_off, &point)) {
        poly->points[poly->cnt++] = point;
      }
    }
    for (int i = max_idx; i >= 0 && poly->cnt < K230_OVERLAY_VERTICES; --i) {
      K230Point point;
      if (project(xs[i], ys[i] + y_off, zs[i] + z_off, &point)) {
        poly->points[poly->cnt++] = point;
      }
    }
  }

  void build_blindspot_poly(const cereal::ModelDataV2::XYZTData::Reader &line, bool left,
                            int max_idx, K230Poly *poly) const {
    poly->cnt = 0;
    const auto xs = line.getX();
    const auto ys = line.getY();
    const auto zs = line.getZ();
    const float first_y_off = left ? 2.8f : 0.01f;
    const float second_y_off = left ? -0.01f : 2.8f;

    for (int i = 0; i <= max_idx && poly->cnt < K230_OVERLAY_VERTICES; ++i) {
      K230Point point;
      if (project(xs[i], ys[i] - first_y_off, zs[i], &point)) {
        poly->points[poly->cnt++] = point;
      }
    }
    for (int i = max_idx; i >= 0 && poly->cnt < K230_OVERLAY_VERTICES; --i) {
      K230Point point;
      if (project(xs[i], ys[i] + second_y_off, zs[i], &point)) {
        poly->points[poly->cnt++] = point;
      }
    }
  }

  float line_z_at_distance(const cereal::ModelDataV2::XYZTData::Reader &line, float distance) const {
    const int idx = path_length_idx(line, distance);
    const auto zs = line.getZ();
    return idx < static_cast<int>(zs.size()) ? finite_or_zero(zs[idx]) : 0.0f;
  }

  bool model_poly_ok(const K230Poly &poly, bool required) const {
    if (poly.cnt < 3) return !required;

    float min_y = poly.points[0].y;
    float max_y = poly.points[0].y;
    float min_x = poly.points[0].x;
    float max_x = poly.points[0].x;
    for (int i = 1; i < poly.cnt; ++i) {
      if (!std::isfinite(poly.points[i].x) || !std::isfinite(poly.points[i].y)) return false;
      min_y = std::min(min_y, poly.points[i].y);
      max_y = std::max(max_y, poly.points[i].y);
      min_x = std::min(min_x, poly.points[i].x);
      max_x = std::max(max_x, poly.points[i].x);
    }

    return min_y >= K230_MODEL_TOP_GUARD_PX &&
           max_y >= 0.0f && min_y <= static_cast<float>(overlay_buffer->height) &&
           max_x >= 0.0f && min_x <= static_cast<float>(overlay_buffer->width);
  }

  bool draw_model_poly(const K230Poly &poly, uint32_t color, bool required = false) {
    if (!model_poly_ok(poly, required)) return false;
    if (poly.cnt >= 3) fill_poly(poly, color);
    return true;
  }

  bool draw_stop_line(const cereal::ModelDataV2::StopLineData::Reader &line) {
    if (!show_stop_line || !hud.long_plan_seen || hud.stopline_distance <= 3.0f) return true;

    const float prob = finite_or_zero(line.getProb());
    if (prob <= 0.5f) return true;

    K230Poly poly = {};
    const float x = finite_or_zero(line.getX());
    const float y = finite_or_zero(line.getY());
    const float z = finite_or_zero(line.getZ()) + 1.22f;
    constexpr float x_off = 0.5f;
    constexpr float y_off = 2.0f;
    const std::array<std::array<float, 2>, 4> corners = {{{x + x_off, y - y_off},
                                                          {x + x_off, y + y_off},
                                                          {x - x_off, y + y_off},
                                                          {x - x_off, y - y_off}}};
    for (const auto &corner : corners) {
      if (project(corner[0], corner[1], z, &poly.points[poly.cnt])) {
        ++poly.cnt;
      }
    }
    return draw_model_poly(poly, argb(static_cast<uint8_t>(std::clamp(prob, 0.0f, 1.0f) * 210.0f), 230, 20, 25));
  }

  void draw_lead_marker(const cereal::RadarState::LeadData::Reader &lead, int label,
                        const cereal::ModelDataV2::XYZTData::Reader &path) {
    if (!lead.getStatus()) return;

    const float d_rel = finite_or_zero(lead.getDRel());
    const float v_rel = finite_or_zero(lead.getVRel());
    if (d_rel <= 0.0f) return;

    K230Point center;
    const float z = line_z_at_distance(path, d_rel) + 1.22f;
    if (!project(d_rel, -finite_or_zero(lead.getYRel()), z, &center)) return;

    const float size = std::clamp((18.0f * 30.0f) / (d_rel / 3.0f + 30.0f), 8.0f, 18.0f);
    const float x = std::clamp(center.x, size * 1.4f, static_cast<float>(overlay_buffer->width) - size * 1.4f);
    const float y = std::min(center.y, static_cast<float>(overlay_buffer->height) - size * 0.6f);
    const bool close_or_closing = d_rel < 18.0f || v_rel < -1.0f || hud.fcw;
    const uint32_t glow = close_or_closing ? argb(120, 255, 220, 60) : argb(90, 60, 255, 90);
    const uint8_t alpha = static_cast<uint8_t>(std::clamp(0.35f + (40.0f - std::min(d_rel, 40.0f)) / 40.0f, 0.35f, 1.0f) * 230.0f);
    const uint32_t fill = close_or_closing ? argb(alpha, 230, 45, 50) : argb(alpha, 110, 240, 95);

    K230Poly poly = {};
    poly.cnt = 3;
    poly.points[0] = {x + size * 1.35f, y + size};
    poly.points[1] = {x, y - size * 0.25f};
    poly.points[2] = {x - size * 1.35f, y + size};
    fill_poly(poly, glow);

    poly.points[0] = {x + size * 1.15f, y + size * 0.9f};
    poly.points[1] = {x, y};
    poly.points[2] = {x - size * 1.15f, y + size * 0.9f};
    fill_poly(poly, fill);

    draw_text_center(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y + size + 2.0f)),
                     label == 1 ? "1" : "2", 1, argb(220, 255, 255, 255), 20);
  }

  uint8_t clamp_byte(float value) const {
    return static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
  }

  uint32_t path_color() const {
    if (!hud.enabled) {
      return argb(125, 255, 255, 255);
    }
    if (hud.steering_pressed) {
      return argb(95, 30, 30, 30);
    }

    const float torque_scale = std::fabs(hud.output_scale) * 0.9f * 255.0f;
    const uint8_t red = clamp_byte(torque_scale);
    const uint8_t green = clamp_byte(255.0f - torque_scale);
    if (hud.laneless) {
      return argb(150, red, 150, green);
    }
    return argb(150, red, green, 0);
  }

  uint32_t lane_color(float prob) const {
    float red = 255.0f;
    float green = 255.0f;
    if (prob > 0.4f) {
      red = (1.0f - ((prob - 0.4f) * 2.5f)) * 255.0f;
    } else {
      green = (1.0f - ((0.4f - prob) * 2.5f)) * 255.0f;
    }
    const uint8_t alpha = static_cast<uint8_t>(std::clamp(prob, 0.0f, 1.0f) * 230.0f);
    return argb(alpha, clamp_byte(red), clamp_byte(green), 0);
  }

  bool draw_model(const cereal::ModelDataV2::Reader &model) {
    auto position = model.getPosition();
    float max_distance = std::clamp(position.getX()[TRAJECTORY_SIZE - 1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    K230Poly poly;
    int max_idx = path_length_idx(position, max_distance);
    build_line_poly(position, 0.9f, 1.22f, max_idx, &poly);
    if (!draw_model_poly(poly, path_color(), true)) return false;

    const auto lane_lines = model.getLaneLines();
    const auto lane_probs = model.getLaneLineProbs();
    if (!hud.laneless) {
      if (hud.left_blindspot && lane_lines.size() > 1 && lane_probs.size() > 1) {
        max_idx = path_length_idx(lane_lines[1], max_distance);
        build_blindspot_poly(lane_lines[1], true, max_idx, &poly);
        if (!draw_model_poly(poly, argb(static_cast<uint8_t>(std::clamp(1.0f - finite_or_zero(lane_probs[1]), 0.20f, 1.0f) * 210.0f), 255, 0, 0))) return false;
      }
      if (hud.right_blindspot && lane_lines.size() > 2 && lane_probs.size() > 2) {
        max_idx = path_length_idx(lane_lines[2], max_distance);
        build_blindspot_poly(lane_lines[2], false, max_idx, &poly);
        if (!draw_model_poly(poly, argb(static_cast<uint8_t>(std::clamp(1.0f - finite_or_zero(lane_probs[2]), 0.20f, 1.0f) * 210.0f), 255, 0, 0))) return false;
      }

      for (int i = 0; i < 4; ++i) {
        const float prob = finite_or_zero(lane_probs[i]);
        if (prob < 0.05f) continue;
        max_idx = path_length_idx(lane_lines[i], max_distance);
        build_line_poly(lane_lines[i], std::max(0.015f, 0.025f * prob), 0.0f, max_idx, &poly);
        if (!draw_model_poly(poly, lane_color(prob))) return false;
      }

      const auto road_edges = model.getRoadEdges();
      const auto edge_stds = model.getRoadEdgeStds();
      for (int i = 0; i < 2; ++i) {
        const float edge_alpha = std::clamp(1.0f - finite_or_zero(edge_stds[i]), 0.0f, 1.0f);
        if (edge_alpha < 0.05f) continue;
        max_idx = path_length_idx(road_edges[i], max_distance);
        build_line_poly(road_edges[i], 0.025f, 0.0f, max_idx, &poly);
        if (!draw_model_poly(poly, argb(static_cast<uint8_t>(edge_alpha * 200.0f), 255, 60, 60))) return false;
      }
    }

    if (!draw_stop_line(model.getStopLine())) return false;

    if (sm.rcv_frame("radarState") > 0) {
      auto radar = sm["radarState"].getRadarState();
      auto lead_one = radar.getLeadOne();
      auto lead_two = radar.getLeadTwo();
      if (lead_one.getStatus()) {
        draw_lead_marker(lead_one, 1, position);
      }
      if (lead_two.getStatus() && (!lead_one.getStatus() || std::fabs(lead_one.getDRel() - lead_two.getDRel()) > 3.0f)) {
        draw_lead_marker(lead_two, 2, position);
      }
    }
    return true;
  }

  template <typename... Args>
  std::string format_text(const char *fmt, Args... args) const {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), fmt, args...);
    return std::string(buffer);
  }

  std::array<uint8_t, 7> glyph_rows(char c) const {
    switch (c) {
      case '0': return {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e};
      case '1': return {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e};
      case '2': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f};
      case '3': return {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e};
      case '4': return {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02};
      case '5': return {0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e};
      case '6': return {0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e};
      case '7': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
      case '8': return {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e};
      case '9': return {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c};
      case 'A': return {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
      case 'B': return {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e};
      case 'C': return {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e};
      case 'D': return {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e};
      case 'E': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f};
      case 'F': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10};
      case 'G': return {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f};
      case 'H': return {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
      case 'I': return {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e};
      case 'J': return {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c};
      case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
      case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f};
      case 'M': return {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11};
      case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
      case 'O': return {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
      case 'P': return {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10};
      case 'Q': return {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d};
      case 'R': return {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11};
      case 'S': return {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e};
      case 'T': return {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
      case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
      case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04};
      case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a};
      case 'X': return {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11};
      case 'Y': return {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04};
      case 'Z': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f};
      case '-': return {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00};
      case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c};
      case ':': return {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00};
      case '/': return {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10};
      case '%': return {0x19, 0x1a, 0x02, 0x04, 0x08, 0x0b, 0x13};
      case '<': return {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02};
      case '>': return {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08};
      case '!': return {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04};
      case '?': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04};
      case '+': return {0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00};
      case '=': return {0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00};
      case '_': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f};
      default: return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
  }

  int text_width(const std::string &text, int scale) const {
    return text.empty() ? 0 : static_cast<int>(text.size()) * 6 * scale - scale;
  }

  std::string clip_text_to_width(const std::string &text, int scale, int max_width) const {
    if (max_width <= 0 || text_width(text, scale) <= max_width) return text;
    std::string clipped = text;
    while (!clipped.empty() && text_width(clipped, scale) > max_width) {
      clipped.pop_back();
    }
    return clipped;
  }

  void fill_rect(int x, int y, int w, int h, uint32_t color, bool mark_dirty = true) {
    if (overlay_buffer == nullptr || w <= 0 || h <= 0) return;

    const int x0 = std::max(0, x);
    const int y0 = std::max(0, y);
    const int x1 = std::min(static_cast<int>(overlay_buffer->width) - 1, x + w - 1);
    const int y1 = std::min(static_cast<int>(overlay_buffer->height) - 1, y + h - 1);
    if (x0 > x1 || y0 > y1) return;

    if (mark_dirty) {
      include_overlay_rect({x0, y0, x1, y1});
    }

    auto *pixels = static_cast<uint32_t *>(overlay_buffer->map);
    const int pitch = static_cast<int>(overlay_buffer->stride / sizeof(uint32_t));
    for (int row = y0; row <= y1; ++row) {
      std::fill(pixels + row * pitch + x0, pixels + row * pitch + x1 + 1, color);
    }
  }

  void draw_text_left(int x, int y, const std::string &raw_text, int scale, uint32_t color, int max_width = 0) {
    if (scale <= 0 || overlay_buffer == nullptr) return;
    const std::string text = clip_text_to_width(raw_text, scale, max_width);
    if (text.empty()) return;

    const int width = text_width(text, scale);
    include_overlay_rect({x, y, x + width - 1, y + 7 * scale - 1});

    int cursor_x = x;
    for (char raw_char : text) {
      char c = raw_char;
      if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
      const auto rows = glyph_rows(c);
      for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
          if ((rows[row] & (1 << (4 - col))) != 0) {
            fill_rect(cursor_x + col * scale, y + row * scale, scale, scale, color, false);
          }
        }
      }
      cursor_x += 6 * scale;
    }
  }

  void draw_text_center(int center_x, int y, const std::string &text, int scale, uint32_t color, int max_width = 0) {
    const std::string clipped = clip_text_to_width(text, scale, max_width);
    draw_text_left(center_x - text_width(clipped, scale) / 2, y, clipped, scale, color);
  }

  void hud_fill_rect(int x, int y, int w, int h, uint32_t color, bool mark_dirty = true) {
    fill_rect(x, y, w, h, color, mark_dirty);
  }

  bool rects_touch(const K230Rect &a, const K230Rect &b) const {
    return a.x0 <= b.x1 + 1 && a.x1 + 1 >= b.x0 &&
           a.y0 <= b.y1 + 1 && a.y1 + 1 >= b.y0;
  }

  void add_merged_rect(std::vector<K230Rect> &rects, K230Rect rect) {
    for (size_t i = 0; i < rects.size();) {
      if (!rects_touch(rects[i], rect)) {
        ++i;
        continue;
      }
      rect.x0 = std::min(rect.x0, rects[i].x0);
      rect.y0 = std::min(rect.y0, rects[i].y0);
      rect.x1 = std::max(rect.x1, rects[i].x1);
      rect.y1 = std::max(rect.y1, rects[i].y1);
      rects.erase(rects.begin() + i);
      i = 0;
    }
    rects.push_back(rect);
  }

  void hud_draw_text_left(int x, int y, const std::string &raw_text, int scale, uint32_t color, int max_width = 0) {
    draw_text_left(x + 2, y + 2, raw_text, scale, argb(185, 0, 0, 0), max_width);
    draw_text_left(x, y, raw_text, scale, color, max_width);
  }

  void hud_draw_text_center(int center_x, int y, const std::string &text, int scale, uint32_t color, int max_width = 0) {
    const std::string clipped = clip_text_to_width(text, scale, max_width);
    const int x = center_x - text_width(clipped, scale) / 2;
    draw_text_left(x + 2, y + 2, clipped, scale, argb(185, 0, 0, 0));
    draw_text_left(x, y, clipped, scale, color);
  }

  void hud_draw_box(int x, int y, int w, int h, uint32_t accent, bool accent_right = false) {
    hud_fill_rect(x, y, w, h, argb(74, 0, 0, 0));
    hud_fill_rect(x, y, w, 1, argb(72, 255, 255, 255));
    hud_fill_rect(x, y + h - 1, w, 1, argb(36, 255, 255, 255));
    hud_fill_rect(accent_right ? x + w - 3 : x, y, 3, h, accent);
  }

  const char *gear_label() const {
    switch (hud.gear) {
      case static_cast<int>(cereal::CarState::GearShifter::PARK): return "P";
      case static_cast<int>(cereal::CarState::GearShifter::REVERSE): return "R";
      case static_cast<int>(cereal::CarState::GearShifter::NEUTRAL): return "N";
      case static_cast<int>(cereal::CarState::GearShifter::DRIVE): return "D";
      case static_cast<int>(cereal::CarState::GearShifter::SPORT): return "S";
      case static_cast<int>(cereal::CarState::GearShifter::LOW): return "L";
      default: return "-";
    }
  }

  std::string gear_text() const {
    if (hud.charge_meter > 0.0f) {
      return format_text("BAT %.0F%%", hud.charge_meter);
    }
    if (hud.gear_step > 0 && hud.gear_step < 9) {
      return format_text("GEAR %s%d", gear_label(), hud.gear_step);
    }
    return format_text("GEAR %s", gear_label());
  }

  bool valid_cruise_speed(float value) const {
    return value > 0.0f && value < 255.0f;
  }

  const char *lateral_method_label() const {
    switch (hud.lateral_control_method) {
      case 0: return "PID";
      case 1: return "INDI";
      case 2: return "LQR";
      case 3: return "TORQ";
      case 4: return "MULTI";
      default: return "--";
    }
  }

  bool tpms_available() const {
    for (float pressure : hud.tpms) {
      if (pressure > 0.1f && pressure < 100.0f) return true;
    }
    return false;
  }

  std::string tpms_text() const {
    if (!tpms_available()) return "TPMS --";
    if (hud.tpms_unit == 2) {
      return format_text("TP %.1F/%.1F %.1F/%.1F", hud.tpms[0], hud.tpms[1], hud.tpms[2], hud.tpms[3]);
    }
    return format_text("TP %.0F/%.0F %.0F/%.0F", hud.tpms[0], hud.tpms[1], hud.tpms[2], hud.tpms[3]);
  }

  const char *bearing_label() const {
    const float bearing = std::fmod(hud.bearing_deg + 360.0f, 360.0f);
    if (bearing <= 22.5f || bearing > 337.5f) return "N";
    if (bearing <= 67.5f) return "NE";
    if (bearing <= 112.5f) return "E";
    if (bearing <= 157.5f) return "SE";
    if (bearing <= 202.5f) return "S";
    if (bearing <= 247.5f) return "SW";
    if (bearing <= 292.5f) return "W";
    return "NW";
  }

  std::string gps_text() const {
    if (!gps_available()) return system_device_text();
    return format_text("GPS %d %.1FM %s%.0F A%.0F", hud.satellite_count, hud.gps_accuracy,
                       bearing_label(), hud.bearing_deg, hud.altitude);
  }

  bool gps_available() const {
    return hud.satellite_count > 0 || hud.gps_accuracy > 0.0f;
  }

  std::string standstill_text() const {
    if (!hud.standstill && hud.standstill_elapsed <= 0.0f) return "";
    const int elapsed = std::max(0, static_cast<int>(std::lround(hud.standstill_elapsed)));
    return format_text("ST %d:%02d", elapsed / 60, elapsed % 60);
  }

  const char *battery_status_label() const {
    if (hud.battery_status.find("Dis") != std::string::npos) return "D";
    if (hud.battery_status.find("Charging") != std::string::npos) return "C";
    if (hud.battery_status.find("Full") != std::string::npos) return "F";
    return "";
  }

  std::string tuning_device_text() const {
    return format_text("SF %.2F M%.0F B%d%s F%d", hud.stiffness_factor, hud.memory_usage,
                       hud.battery_percent, battery_status_label(), hud.fan_speed);
  }

  const char *thermal_status_label() const {
    switch (hud.thermal_status) {
      case 0: return "G";
      case 1: return "Y";
      case 2: return "R";
      case 3: return "D";
      default: return "?";
    }
  }

  const char *network_type_label() const {
    switch (hud.network_type) {
      case 0: return "--";
      case 1: return "WF";
      case 2: return "2G";
      case 3: return "3G";
      case 4: return "LTE";
      case 5: return "5G";
      case 6: return "ETH";
      default: return "?";
    }
  }

  int network_strength_bars() const {
    if (hud.network_strength <= 0) return 0;
    return std::min(hud.network_strength + 1, 5);
  }

  std::string system_device_text() const {
    return format_text("FS%.0F T%s F%d N%s%d", hud.free_space_percent, thermal_status_label(),
                       hud.fan_speed, network_type_label(), network_strength_bars());
  }

  std::string panda_control_text() const {
    if (!hud.panda_seen || !hud.panda_alive) return "PANDA --";
    if (!hud.ignition) return "IGN --";
    return hud.control_allowed ? "CTRL OK" : "CTRL --";
  }

  std::string torque_text() const {
    const char *flag = hud.steer_warning ? "!" : (hud.steering_pressed ? "O" : "");
    return format_text("T%.0F S%.0F R%.0F%s", hud.steering_torque, hud.steer_cmd, hud.engine_rpm, flag);
  }

  float display_speed() const {
    return hud.speed_kph * (is_metric ? 1.0f : 0.6213712f);
  }

  float display_relative_speed(float speed_mps) const {
    return speed_mps * (is_metric ? 3.6f : 2.2369363f);
  }

  const char *speed_unit_label() const {
    return is_metric ? "KPH" : "MPH";
  }

  std::string lead_text() const {
    if (!hud.lead_status) {
      if (hud.radar_distance > 0.0f && hud.radar_distance < 149.0f) {
        return format_text("RAD %.0FM", hud.radar_distance);
      }
      return "LEAD --";
    }
    if (hud.lead2_status) {
      return format_text("L1 %.0F/%+.0F L2 %.0F", hud.lead_d, display_relative_speed(hud.lead_v), hud.lead2_d);
    }
    return format_text("LEAD %.0FM %+.0F", hud.lead_d, display_relative_speed(hud.lead_v));
  }

  const char *longitudinal_source_label() const {
    switch (hud.longitudinal_source) {
      case 0: return "L0";
      case 1: return "L1";
      case 2: return "CR";
      case 3: return "ST";
      default: return "--";
    }
  }

  std::string speed_limit_tr_text() const {
    if (hud.limit_speed_camera > 0.0f || hud.limit_speed_camera_dist > 0.0f) {
      if ((hud.map_sign_cam == 195 || hud.map_sign_cam == 197) && hud.limit_speed_camera <= 0.0f) {
        return format_text("%s%sVAR D%.0F", hud.fcw ? "F " : "", hud.speed_limit_pause ? "P " : "",
                           hud.limit_speed_camera_dist);
      }
      return format_text("%s%sSL %.0F D%.0F", hud.fcw ? "F " : "", hud.speed_limit_pause ? "P " : "",
                         hud.limit_speed_camera, hud.limit_speed_camera_dist);
    }
    if (hud.map_sign != 0 || hud.map_sign_cam != 0) {
      return format_text("%sMS %d/%d", hud.fcw ? "F " : "", hud.map_sign, hud.map_sign_cam);
    }
    return format_text("%s%s %u/%.1F%s%s", hud.fcw ? "F " : "", longitudinal_source_label(),
                       hud.dynamic_tr_mode, hud.dynamic_tr_value,
                       hud.gap_by_speed ? " GB" : "", hud.dm_active ? " DM" : "");
  }

  std::string debug_msg_text() const {
    if (!hud.alert_msg1.empty()) return format_text("D1 %s", hud.alert_msg1.c_str());
    if (!hud.alert_msg2.empty()) return format_text("D2 %s", hud.alert_msg2.c_str());
    if (!hud.alert_msg3.empty()) return format_text("D3 %s", hud.alert_msg3.c_str());
    return "";
  }

  std::string hold_status_text() const {
    const std::string standstill = standstill_text();
    if (hud.brake_hold || !standstill.empty() || hud.driver_acc) {
      return format_text("%s%s%s", hud.brake_hold ? "HLD " : "",
                         standstill.c_str(),
                         hud.driver_acc ? " DRV" : "");
    }
    return debug_msg_text();
  }

  std::string stopline_text() const {
    if (!hud.long_plan_seen || (hud.stopline_distance <= 0.0f && hud.stopline_prob <= 0.0f)) {
      return "STP --";
    }
    return format_text("STP %.0FM P%.1F", hud.stopline_distance, hud.stopline_prob);
  }

  const char *service_token(bool seen, bool alive) const {
    if (alive) return "OK";
    return seen ? "OLD" : "--";
  }

  const char *calib_status_label() const {
    if (!hud.calib_seen) return "--";
    switch (hud.calib_status) {
      case 1: return "OK";
      case 2: return "BAD";
      default: return "CAL";
    }
  }

  std::string calibration_text() const {
    if (!hud.calib_seen) return "CAL --";
    if (!hud.calib_alive) {
      return format_text("CAL OLD%d %.1F/%.1F", hud.calib_percent, hud.calib_pitch_deg, hud.calib_yaw_deg);
    }
    if (hud.calib_status == 2) {
      return format_text("CAL BAD %.1F/%.1F", hud.calib_pitch_deg, hud.calib_yaw_deg);
    }
    return format_text("CAL %d%% %.1F/%.1F", hud.calib_percent, hud.calib_pitch_deg, hud.calib_yaw_deg);
  }

  std::string service_status_text() const {
    if (!hud.calib_seen) {
      return format_text("M%s CTL%s CAR%s PLN%s CAL--",
                         service_token(hud.model_seen, hud.model_alive),
                         service_token(hud.controls_seen, hud.controls_alive),
                         service_token(hud.car_seen, hud.car_alive),
                         service_token(hud.plan_seen, hud.plan_alive));
    }
    return format_text("M%s CTL%s CAR%s PLN%s CAL%s%d",
                       service_token(hud.model_seen, hud.model_alive),
                       service_token(hud.controls_seen, hud.controls_alive),
                       service_token(hud.car_seen, hud.car_alive),
                       service_token(hud.plan_seen, hud.plan_alive),
                       hud.calib_alive ? calib_status_label() : "OLD", hud.calib_percent);
  }

  std::string system_alert_text() const {
    if (!hud.model_seen) return "WAITING FOR MODEL";
    if (!hud.model_alive) return "MODEL STALE";
    if (!hud.controls_seen) return "WAITING FOR CAN CONTROLSD";
    if (!hud.controls_alive) return "CONTROLSD STALE";
    if (!hud.car_seen) return "WAITING FOR CARSTATE";
    if (!hud.car_alive) return "CARSTATE STALE";
    if (!hud.plan_seen) return "WAITING FOR PLANNER";
    if (!hud.plan_alive) return "PLANNER STALE";
    if (!hud.calib_seen) return "WAITING FOR CALIBRATION";
    if (!hud.calib_alive) return "CALIBRATION STALE";
    if (hud.calib_status == 2) return "CALIBRATION INVALID";
    return "";
  }

  bool alert_is_critical() const {
    return hud.alert_status == static_cast<int>(cereal::ControlsState::AlertStatus::CRITICAL) ||
           hud.alert_size >= static_cast<int>(cereal::ControlsState::AlertSize::FULL);
  }

  bool alert_is_warning() const {
    return alert_is_critical() ||
           hud.alert_status == static_cast<int>(cereal::ControlsState::AlertStatus::USER_PROMPT) ||
           hud.alert_size >= static_cast<int>(cereal::ControlsState::AlertSize::MID);
  }

  void draw_hud() {
    if (overlay_buffer == nullptr) return;

    const int width = static_cast<int>(cfg.width);
    const int height = static_cast<int>(cfg.height);
    const uint32_t white = argb(230, 255, 255, 255);
    const uint32_t dim = argb(170, 210, 220, 230);
    const uint32_t green = argb(230, 80, 230, 95);
    const uint32_t blue = argb(230, 90, 170, 255);
    const uint32_t yellow = argb(230, 255, 220, 60);
    const uint32_t orange = argb(235, 255, 150, 50);
    const uint32_t red = argb(235, 255, 70, 70);

    const std::string system_alert = system_alert_text();
    const bool driver_alert = hud.alert_size > 0 || !hud.alert_text1.empty() || !hud.alert_text2.empty();
    const uint32_t status = driver_alert ?
                            (alert_is_critical() ? red : (alert_is_warning() ? orange : yellow)) :
                            (!system_alert.empty() ? orange : (hud.enabled ? green : (hud.engageable ? blue : argb(220, 110, 120, 130))));

    hud_fill_rect(0, 0, width, 6, status);

    const uint32_t speed_color = hud.brake_pressed || hud.brake_lights ? red :
                                 (hud.gas_pressed ? green :
                                  (hud.accel < -0.2f || hud.a_req < -0.2f ? orange : white));
    const std::string set_speed = valid_cruise_speed(hud.v_cruise) ?
                                  format_text("%d", static_cast<int>(std::lround(hud.v_cruise))) : "---";
    const std::string cruise_speed = valid_cruise_speed(hud.v_set_dis) ?
                                     format_text("%d", static_cast<int>(std::lround(hud.v_set_dis))) : "---";

    const int box_w = 236;
    const int left_box_x = 8;
    const int right_box_x = width - box_w - 8;
    const int left_x = left_box_x + 10;
    const int right_x = right_box_x + 10;
    const int col_w = box_w - 20;
    const int row_step = 18;

    hud_draw_text_center(width / 2, 12, format_text("%d", static_cast<int>(std::lround(display_speed()))), 8, speed_color, 220);
    hud_draw_text_center(width / 2, 80, speed_unit_label(), 2, dim, 140);

    hud_draw_box(left_box_x, 10, box_w, 104, status);
    hud_draw_text_left(left_x, 14, "CRUISE", 1, dim, col_w);
    hud_draw_text_left(left_x, 34, format_text("SET %s", set_speed.c_str()), 3, white, col_w);
    hud_draw_text_left(left_x, 72, format_text("CRZ %s%s", cruise_speed.c_str(), hud.cruise_acc ? " ACC" : ""), 2, dim, col_w);

    hud_draw_box(right_box_x, 10, box_w, 104, status, true);
    hud_draw_text_left(right_x, 14, "OPENPILOT", 1, dim, col_w);
    hud_draw_text_left(right_x, 30, hud.active ? "OP ACT" : (hud.enabled ? "OP EN" : (hud.engageable ? "OP RDY" : "OP OFF")), 2,
                       hud.enabled ? green : dim, col_w);
    hud_draw_text_left(right_x, 54, format_text("GAP %u %s", hud.cruise_gap, gear_text().c_str()), 2, dim, col_w);
    const bool panda_control_ok = hud.panda_alive && hud.ignition && hud.control_allowed;
    const uint32_t panda_control_color = panda_control_ok ? green : ((!hud.panda_alive || !hud.ignition) ? orange : dim);
    hud_draw_text_left(right_x, 78, panda_control_text(), 2, panda_control_color, col_w);
    const bool core_services_alive = hud.model_alive && hud.controls_alive && hud.car_alive && hud.plan_alive && hud.calib_alive;
    hud_draw_text_left(right_x, 96, service_status_text(), 1, core_services_alive ? dim : orange, col_w);

    if (hud.left_blinker) hud_draw_text_left(left_box_x + box_w + 8, 34, "<", 4, yellow);
    if (hud.right_blinker) hud_draw_text_left(right_box_x - 38, 34, ">", 4, yellow);
    if (hud.left_blindspot) hud_fill_rect(0, 96, 10, height - 116, red);
    if (hud.right_blindspot) hud_fill_rect(width - 10, 96, 10, height - 116, red);

    const uint32_t gps_system_color = gps_available() ? dim :
                                      (hud.thermal_status >= 2 ? red : (hud.thermal_status == 1 ? orange : dim));

    if (hud_mode != K230_PREVIEW_HUD_MINIMAL) {
      hud_draw_box(left_box_x, 124, box_w, 126, blue);
      hud_draw_text_left(left_x, 128, "ROAD", 1, dim, col_w);
      int left_y = 144;
      hud_draw_text_left(left_x, left_y, format_text("STR %.1F/%.1F", hud.steering_angle, hud.desired_angle), 2, white, col_w);
      left_y += row_step;
      hud_draw_text_left(left_x, left_y, lead_text(), 2, hud.lead_status ? white : dim, col_w);
      left_y += row_step;
      hud_draw_text_left(left_x, left_y, format_text("LANE %.2F/%.2F", hud.l_prob, hud.r_prob), 2, white, col_w);
      left_y += row_step;
      hud_draw_text_left(left_x, left_y, format_text("LW %.1FM DP %.2F", hud.lane_width, hud.d_prob), 2, dim, col_w);
      left_y += row_step;
      hud_draw_text_left(left_x, left_y, format_text("M %.0F R%.2F/%.2F", hud.model_execution_ms,
                         hud.road_edge_confs[0], hud.road_edge_confs[1]), 2, dim, col_w);
      left_y += row_step;
      hud_draw_text_left(left_x, left_y, format_text("L %.1F %.1F %.1F %.1F", hud.lane_line_probs[0],
                         hud.lane_line_probs[1], hud.lane_line_probs[2], hud.lane_line_probs[3]), 2, dim, col_w);

      hud_draw_box(right_box_x, 124, box_w, 126, green, true);
      hud_draw_text_left(right_x, 128, "SYSTEM", 1, dim, col_w);
      int right_y = 144;
      const uint32_t cpu_color = (hud.cpu_usage >= 90.0f || hud.cpu_temp >= 85.0f) ? red :
                                 (hud.cpu_usage >= 75.0f || hud.cpu_temp >= 75.0f) ? orange :
                                 (hud.cpu_usage >= 60.0f || hud.cpu_temp >= 65.0f) ? yellow : dim;
      hud_draw_text_left(right_x, right_y, format_text("CPU %.0F%% %.0FC", hud.cpu_usage, hud.cpu_temp), 2,
                         cpu_color, col_w);
      right_y += row_step;
      hud_draw_text_left(right_x, right_y, gps_text(), 2, gps_system_color, col_w);
      right_y += row_step;
      const uint32_t calib_color = (!hud.calib_seen || !hud.calib_alive) ? orange :
                                   (hud.calib_status == 2 ? red : (hud.calib_percent < 100 ? yellow : dim));
      hud_draw_text_left(right_x, right_y, calibration_text(), 2, calib_color, col_w);
      right_y += row_step;
      hud_draw_text_left(right_x, right_y, format_text("%s SC %.2F S %.0F", hud.longitudinal_control ? "LNG" : "LAT",
                         hud.output_scale, hud.safety_speed), 2, dim, col_w);
      right_y += row_step;
      hud_draw_text_left(right_x, right_y, speed_limit_tr_text(), 2, hud.speed_limit_pause ? orange : dim, col_w);
      right_y += row_step;
      hud_draw_text_left(right_x, right_y, hold_status_text(), 2, dim, col_w);

      if (hud_mode == K230_PREVIEW_HUD_FULL) {
        hud_draw_box(left_box_x, 262, box_w, 92, yellow);
        hud_draw_text_left(left_x, 266, "VEHICLE", 1, dim, col_w);
        left_y = 282;
        hud_draw_text_left(left_x, left_y, format_text("ACC %.2F", hud.accel), 2, dim, col_w);
        left_y += row_step;
        hud_draw_text_left(left_x, left_y, format_text("BS %s/%s", hud.left_blindspot ? "L" : "-", hud.right_blindspot ? "R" : "-"), 2,
                           (hud.left_blindspot || hud.right_blindspot) ? red : dim, col_w);
        left_y += row_step;
        hud_draw_text_left(left_x, left_y, tpms_text(), 2, tpms_available() ? dim : argb(130, 180, 185, 190), col_w);
        left_y += row_step;
        hud_draw_text_left(left_x, left_y, torque_text(), 2, hud.steer_warning ? red : (hud.steering_pressed ? orange : dim), col_w);

        hud_draw_box(right_box_x, 262, box_w, 92, yellow, true);
        hud_draw_text_left(right_x, 266, "TUNE", 1, dim, col_w);
        right_y = 282;
        hud_draw_text_left(right_x, right_y, format_text("SR %.1F AO %.2F", hud.steer_ratio, hud.angle_offset_avg), 2, dim, col_w);
        right_y += row_step;
        hud_draw_text_left(right_x, right_y, format_text("%s TCO %.2F", hud.laneless ? "LANELESS" : "LANE",
                           -hud.total_camera_offset), 2, dim, col_w);
        right_y += row_step;
        hud_draw_text_left(right_x, right_y, tuning_device_text(), 2, dim, col_w);
        right_y += row_step;
        hud_draw_text_left(right_x, right_y, format_text("M %s AD %.2F %s", lateral_method_label(), hud.steer_actuator_delay,
                           stopline_text().c_str()), 2, dim, col_w);
      }
    }

    if (driver_alert || !system_alert.empty()) {
      const uint32_t alert_text_color = driver_alert && alert_is_critical() ? red : orange;
      const int alert_w = 520;
      const int alert_x = (width - alert_w) / 2;
      const int alert_y = height - 58;
      hud_draw_box(alert_x, alert_y, alert_w, 50, alert_text_color);
      hud_draw_text_center(width / 2, alert_y + 8, driver_alert ? (hud.alert_text1.empty() ? "ALERT" : hud.alert_text1) : system_alert,
                           3, alert_text_color, alert_w - 18);
      hud_draw_text_center(width / 2, alert_y + 36, driver_alert ? (hud.alert_text2.empty() ? hud.alert_type : hud.alert_text2) : service_status_text(),
                           1, white, alert_w - 18);
    }

  }

  void fill_poly(const K230Poly &poly, uint32_t color) {
    if (poly.cnt < 3) return;

    auto *pixels = static_cast<uint32_t *>(overlay_buffer->map);
    const int width = static_cast<int>(overlay_buffer->stride / 4);
    const int height = static_cast<int>(overlay_buffer->height);

    float min_y = poly.points[0].y;
    float max_y = poly.points[0].y;
    float min_x = poly.points[0].x;
    float max_x = poly.points[0].x;
    for (int i = 1; i < poly.cnt; ++i) {
      min_y = std::min(min_y, poly.points[i].y);
      max_y = std::max(max_y, poly.points[i].y);
      min_x = std::min(min_x, poly.points[i].x);
      max_x = std::max(max_x, poly.points[i].x);
    }

    const int y0 = std::max(0, static_cast<int>(std::floor(min_y)));
    const int y1 = std::min(height - 1, static_cast<int>(std::ceil(max_y)));
    const int dirty_x0 = std::max(0, static_cast<int>(std::floor(min_x)));
    const int dirty_x1 = std::min(static_cast<int>(overlay_buffer->width) - 1, static_cast<int>(std::ceil(max_x)));
    include_overlay_rect({dirty_x0, y0, dirty_x1, y1});

    std::array<float, K230_OVERLAY_VERTICES> intersections = {};

    for (int y = y0; y <= y1; ++y) {
      const float scan_y = static_cast<float>(y) + 0.5f;
      int count = 0;
      for (int i = 0, j = poly.cnt - 1; i < poly.cnt; j = i++) {
        const K230Point &a = poly.points[i];
        const K230Point &b = poly.points[j];
        if ((a.y <= scan_y && b.y > scan_y) || (b.y <= scan_y && a.y > scan_y)) {
          const float t = (scan_y - a.y) / (b.y - a.y);
          intersections[count++] = a.x + t * (b.x - a.x);
        }
      }
      if (count < 2) continue;
      std::sort(intersections.begin(), intersections.begin() + count);
      for (int i = 0; i + 1 < count; i += 2) {
        const int x0 = std::max(0, static_cast<int>(std::floor(intersections[i])));
        const int x1 = std::min(static_cast<int>(overlay_buffer->width) - 1,
                                static_cast<int>(std::ceil(intersections[i + 1])));
        for (int x = x0; x <= x1; ++x) {
          pixels[y * width + x] = color;
        }
      }
    }
  }

  K230PreviewConfig cfg;
  struct display *display = nullptr;
  Params params;
  SubMaster sm;
  struct v4l2_drm_context ctx = {};
  struct display_plane *video_plane = nullptr;
  std::vector<display_buffer *> video_buffers;
  struct display_plane *overlay_plane = nullptr;
  struct display_buffer *overlay_buffer = nullptr;
  std::vector<display_buffer *> overlay_buffers;
  std::vector<std::vector<K230Rect>> overlay_dirty_rects;
  std::vector<K230Rect> current_overlay_dirty_rects;
  K230Mat3 view_from_calib = {};
  K230HudState hud;
  uint64_t last_model_frame = 0;
  uint64_t overlay_frame_counter = 0;
  uint64_t last_param_check_frame = 0;
  size_t video_buffer_index = 0;
  size_t overlay_buffer_index = 0;
  int hud_mode = K230_PREVIEW_HUD_FULL;
  bool is_metric = true;
  bool show_stop_line = false;
  bool setup = false;
  bool first_overlay = true;
  bool pending_hud_update = true;
};

}  // namespace

int main() {
  try {
    K230PreviewConfig cfg = read_config();
    struct display *display = display_init(0);
    if (display == nullptr) {
      throw std::runtime_error("display_init failed for K230 previewd");
    }
    display->drm_rotation = rotation_90;

    if (display->width >= display->height) {
      throw std::runtime_error("K230 previewd expects a portrait DRM mode");
    }
    cfg.width = display->height;
    cfg.height = display->width;

    K230PreviewRuntime preview(cfg, display);
    return preview.run();
  } catch (const std::exception &e) {
    LOGE("K230 previewd error: %s", e.what());
    return 1;
  }
}
