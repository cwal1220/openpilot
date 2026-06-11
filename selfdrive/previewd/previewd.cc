#include <linux/videodev2.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/k230_vvcam.h"
#include "selfdrive/common/modeldata.h"
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

constexpr float K230_OV5647_FX = 1625.7416788144435f;
constexpr float K230_OV5647_FY = 1585.9830269782024f;
constexpr float K230_OV5647_CX = 946.13450988811394f;
constexpr float K230_OV5647_CY = 537.34063862123787f;
constexpr int K230_PREVIEW_RETRY_MS = 300;
constexpr uint32_t K230_PREVIEW_TIMEOUTS_BEFORE_RESTART = 3;

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
  bool longitudinal_control = false;

  float speed_kph = 0.0f;
  float v_cruise = 0.0f;
  float safety_speed = 0.0f;
  float v_set_dis = 0.0f;
  float steering_angle = 0.0f;
  float desired_angle = 0.0f;
  float accel = 0.0f;
  float a_req = 0.0f;
  float output_scale = 0.0f;
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
  float lead_d = 0.0f;
  float lead_v = 0.0f;
  float steering_torque = 0.0f;
  float engine_rpm = 0.0f;
  float charge_meter = 0.0f;
  float steer_actuator_delay = 0.0f;
  float gps_accuracy = 0.0f;
  float altitude = 0.0f;
  float model_execution_ms = 0.0f;
  float total_camera_offset = 0.0f;
  float standstill_elapsed = 0.0f;
  std::array<float, 4> lane_line_probs = {};
  std::array<float, 2> road_edge_confs = {};
  std::array<float, 4> tpms = {};

  uint8_t cruise_gap = 0;
  uint8_t lateral_control_method = 0;
  int tpms_unit = 0;
  int gear_step = 0;
  int satellite_count = 0;
  int alert_size = 0;
  int gear = 0;
  std::string alert_text1;
  std::string alert_text2;
  std::string alert_type;
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
            "ubloxGnss", "gpsLocationExternal"}) {
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
      rotate_nv12_90_tiled(src, video_buffers[video_buffer_index]);
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

    LOGW("K230 previewd opening /dev/video%d %ux%u NV12 cpu-rotate-to %ux%u",
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

    for (int i = 0; i < 3; ++i) {
      display_buffer *buffer = display_allocate_buffer(video_plane, display->width, display->height);
      if (buffer == nullptr) {
        throw std::runtime_error("K230 previewd could not allocate NV12 display buffer");
      }
      const size_t y_size = static_cast<size_t>(buffer->stride) * buffer->height;
      std::memset(buffer->map, 16, y_size);
      std::memset(static_cast<uint8_t *>(buffer->map) + y_size, 128, buffer->size - y_size);
      video_buffers.push_back(buffer);
    }
  }

  void rotate_nv12_90_tiled(const uint8_t *src, display_buffer *dst) {
    const int src_w = static_cast<int>(cfg.width);
    const int src_h = static_cast<int>(cfg.height);
    const int dst_h = static_cast<int>(dst->height);
    auto *dst_y = static_cast<uint8_t *>(dst->map);
    auto *dst_uv = dst_y + dst->stride * dst_h;
    const uint8_t *src_y = src;
    const uint8_t *src_uv = src + src_w * src_h;

    constexpr int tile_w = 32;
    constexpr int tile_h = 32;
    for (int y0 = 0; y0 < src_h; y0 += tile_h) {
      const int y1 = std::min(y0 + tile_h, src_h);
      for (int x0 = 0; x0 < src_w; x0 += tile_w) {
        const int x1 = std::min(x0 + tile_w, src_w);
        for (int x = x0; x < x1; ++x) {
          uint8_t *d = dst_y + x * dst->stride + (src_h - y1);
          for (int y = y1 - 1; y >= y0; --y) {
            *d++ = src_y[y * src_w + x];
          }
        }
      }
    }

    const int src_ch = src_h / 2;
    const int src_cw = src_w / 2;
    constexpr int uv_tile_w = 32;
    constexpr int uv_tile_h = 16;
    for (int cy0 = 0; cy0 < src_ch; cy0 += uv_tile_h) {
      const int cy1 = std::min(cy0 + uv_tile_h, src_ch);
      for (int cx0 = 0; cx0 < src_cw; cx0 += uv_tile_w) {
        const int cx1 = std::min(cx0 + uv_tile_w, src_cw);
        for (int cx = cx0; cx < cx1; ++cx) {
          uint8_t *d = dst_uv + cx * dst->stride + (src_ch - cy1) * 2;
          for (int cy = cy1 - 1; cy >= cy0; --cy) {
            const uint8_t *s = src_uv + cy * src_w + cx * 2;
            *d++ = s[0];
            *d++ = s[1];
          }
        }
      }
    }
  }

  void setup_overlay() {
    overlay_plane = display_get_plane(display, DRM_FORMAT_ARGB8888);
    if (overlay_plane == nullptr) {
      LOGE("K230 previewd could not allocate ARGB overlay plane");
      return;
    }

    for (int i = 0; i < 2; ++i) {
      display_buffer *buffer = display_allocate_buffer(overlay_plane, display->width, display->height);
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
    if (sm.updated("liveCalibration")) {
      auto rpy = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
      if (rpy.size() >= 3 && std::isfinite(rpy[0]) && std::isfinite(rpy[1]) && std::isfinite(rpy[2])) {
        view_from_calib = view_from_calib_from_rpy(rpy[0], rpy[1], rpy[2]);
        changed = true;
      }
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
      hud.alert_size = static_cast<int>(cs.getAlertSize());
      hud.alert_text1 = sanitize_text(cs.getAlertText1().cStr(), 32);
      hud.alert_text2 = sanitize_text(cs.getAlertText2().cStr(), 32);
      hud.alert_type = sanitize_text(cs.getAlertType().cStr(), 24);
      changed = true;
    }

    if (sm.updated("carState")) {
      auto car = sm["carState"].getCarState();
      hud.speed_kph = std::max(0.0f, finite_or_zero(car.getVEgo() * 3.6f));
      hud.v_set_dis = finite_or_zero(car.getVSetDis());
      hud.cruise_gap = car.getCruiseGapSet();
      hud.steering_angle = finite_or_zero(car.getSteeringAngleDeg());
      hud.brake_pressed = car.getBrakePressed();
      hud.brake_lights = car.getBrakeLights();
      hud.gas_pressed = car.getGasPressed();
      hud.left_blinker = car.getLeftBlinker();
      hud.right_blinker = car.getRightBlinker();
      hud.left_blindspot = car.getLeftBlindspot();
      hud.right_blindspot = car.getRightBlindspot();
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
      changed = true;
    }

    if (sm.updated("lateralPlan")) {
      auto lp = sm["lateralPlan"].getLateralPlan();
      hud.lane_width = finite_or_zero(lp.getLaneWidth());
      hud.d_prob = finite_or_zero(lp.getDProb());
      hud.l_prob = finite_or_zero(lp.getLProb());
      hud.r_prob = finite_or_zero(lp.getRProb());
      hud.output_scale = finite_or_zero(lp.getOutputScale());
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
      auto lead = sm["radarState"].getRadarState().getLeadOne();
      hud.lead_status = lead.getStatus();
      hud.lead_d = finite_or_zero(lead.getDRel());
      hud.lead_v = finite_or_zero(lead.getVRel());
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
      for (const auto &panda_state : panda_states) {
        hud.control_allowed = hud.control_allowed || panda_state.getControlsAllowed();
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
      changed = true;
    }

    return changed;
  }

  void update_overlay() {
    const bool state_changed = update_state();
    if (overlay_buffers.empty()) return;

    const uint64_t model_frame = sm.rcv_frame("modelV2");
    ++overlay_frame_counter;
    pending_hud_update = pending_hud_update || state_changed;
    const bool hud_redraw = pending_hud_update && (first_overlay || overlay_frame_counter % 3 == 0);
    if (!first_overlay && !hud_redraw && model_frame == last_model_frame) {
      return;
    }

    overlay_buffer_index = (overlay_buffer_index + 1) % overlay_buffers.size();
    overlay_buffer = overlay_buffers[overlay_buffer_index];
    clear_overlay_rect(overlay_buffer, overlay_dirty_rects[overlay_buffer_index]);
    current_overlay_dirty = {};
    if (model_frame > 0) {
      draw_model(sm["modelV2"].getModelV2());
    }
    draw_hud();
    overlay_dirty_rects[overlay_buffer_index] = current_overlay_dirty;
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

  void include_overlay_rect(const K230Rect &rect) {
    if (!rect_valid(rect)) return;
    if (!rect_valid(current_overlay_dirty)) {
      current_overlay_dirty = rect;
      return;
    }
    current_overlay_dirty.x0 = std::min(current_overlay_dirty.x0, rect.x0);
    current_overlay_dirty.y0 = std::min(current_overlay_dirty.y0, rect.y0);
    current_overlay_dirty.x1 = std::max(current_overlay_dirty.x1, rect.x1);
    current_overlay_dirty.y1 = std::max(current_overlay_dirty.y1, rect.y1);
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
    if (std::fabs(ez) < 1e-6f) return false;

    const float px = (fx * ex + cx * ez) / ez;
    const float py = (fy * ey + cy * ez) / ez;
    if (!std::isfinite(px) || !std::isfinite(py) ||
        px < -200.0f || px > cfg.width + 200.0f ||
        py < -200.0f || py > cfg.height + 200.0f) {
      return false;
    }

    *out = {static_cast<float>(cfg.height) - py, px};
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

  void draw_model(const cereal::ModelDataV2::Reader &model) {
    auto position = model.getPosition();
    float max_distance = std::clamp(position.getX()[TRAJECTORY_SIZE - 1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    K230Poly poly;
    int max_idx = path_length_idx(position, max_distance);
    build_line_poly(position, 0.9f, 1.22f, max_idx, &poly);
    fill_poly(poly, argb(130, 80, 220, 80));

    const auto lane_lines = model.getLaneLines();
    const auto lane_probs = model.getLaneLineProbs();
    for (int i = 0; i < 4; ++i) {
      if (lane_probs[i] < 0.05f) continue;
      max_idx = path_length_idx(lane_lines[i], max_distance);
      build_line_poly(lane_lines[i], std::max(0.015f, 0.025f * lane_probs[i]), 0.0f, max_idx, &poly);
      const uint8_t alpha = static_cast<uint8_t>(std::clamp(lane_probs[i], 0.0f, 1.0f) * 220.0f);
      fill_poly(poly, argb(alpha, 245, 245, 245));
    }

    const auto road_edges = model.getRoadEdges();
    const auto edge_stds = model.getRoadEdgeStds();
    for (int i = 0; i < 2; ++i) {
      const float edge_alpha = std::clamp(1.0f - edge_stds[i], 0.0f, 1.0f);
      if (edge_alpha < 0.05f) continue;
      max_idx = path_length_idx(road_edges[i], max_distance);
      build_line_poly(road_edges[i], 0.025f, 0.0f, max_idx, &poly);
      fill_poly(poly, argb(static_cast<uint8_t>(edge_alpha * 200.0f), 255, 60, 60));
    }
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

  std::string gps_text() const {
    if (hud.satellite_count <= 0 && hud.gps_accuracy <= 0.0f) return "GPS --";
    return format_text("GPS %d %.1FM %.0FM", hud.satellite_count, hud.gps_accuracy, hud.altitude);
  }

  std::string standstill_text() const {
    if (!hud.standstill && hud.standstill_elapsed <= 0.0f) return "";
    const int elapsed = std::max(0, static_cast<int>(std::lround(hud.standstill_elapsed)));
    return format_text("ST %d:%02d", elapsed / 60, elapsed % 60);
  }

  void draw_hud() {
    if (overlay_buffer == nullptr) return;

    const int width = static_cast<int>(overlay_buffer->width);
    const int height = static_cast<int>(overlay_buffer->height);
    const uint32_t white = argb(230, 255, 255, 255);
    const uint32_t dim = argb(170, 210, 220, 230);
    const uint32_t green = argb(230, 80, 230, 95);
    const uint32_t blue = argb(230, 90, 170, 255);
    const uint32_t yellow = argb(230, 255, 220, 60);
    const uint32_t orange = argb(235, 255, 150, 50);
    const uint32_t red = argb(235, 255, 70, 70);

    const bool alert_active = hud.alert_size > 0 || !hud.alert_text1.empty() || !hud.alert_text2.empty();
    const uint32_t status = alert_active ? red : (hud.enabled ? green : (hud.engageable ? blue : argb(220, 110, 120, 130)));

    fill_rect(0, 0, width, 122, argb(100, 0, 0, 0));
    fill_rect(0, 0, width, 6, status);

    const uint32_t speed_color = hud.brake_pressed || hud.brake_lights ? red :
                                 (hud.gas_pressed ? green :
                                  (hud.accel < -0.2f || hud.a_req < -0.2f ? orange : white));
    draw_text_center(width / 2, 18, format_text("%d", static_cast<int>(std::lround(hud.speed_kph))), 8, speed_color, 170);
    draw_text_center(width / 2, 84, "KPH", 2, dim);

    const std::string set_speed = valid_cruise_speed(hud.v_cruise) ?
                                  format_text("%d", static_cast<int>(std::lround(hud.v_cruise))) : "---";
    const std::string cruise_speed = valid_cruise_speed(hud.v_set_dis) ?
                                     format_text("%d", static_cast<int>(std::lround(hud.v_set_dis))) : "---";
    draw_text_left(12, 18, "SET", 2, dim);
    draw_text_left(12, 42, set_speed, 4, white, 100);
    draw_text_left(12, 86, format_text("CRZ %s%s", cruise_speed.c_str(), hud.cruise_acc ? " ACC" : ""), 2, dim, 135);

    draw_text_left(width - 138, 18, hud.active ? "OP ACT" : (hud.enabled ? "OP EN" : (hud.engageable ? "OP RDY" : "OP OFF")), 2,
                   hud.enabled ? green : dim, 128);
    draw_text_left(width - 138, 44, format_text("GAP %u", hud.cruise_gap), 2, dim, 128);
    draw_text_left(width - 138, 70, gear_text(), 2, dim, 128);
    draw_text_left(width - 138, 96, hud.control_allowed ? "CTRL OK" : "CTRL --", 2,
                   hud.control_allowed ? green : dim, 128);

    if (hud.left_blinker) draw_text_left(132, 46, "<", 5, yellow);
    if (hud.right_blinker) draw_text_left(width - 160, 46, ">", 5, yellow);
    if (hud.left_blindspot) fill_rect(0, 150, 12, height - 330, red);
    if (hud.right_blindspot) fill_rect(width - 12, 150, 12, height - 330, red);

    const int panel_h = 196;
    const int panel_y = height - panel_h;
    fill_rect(0, panel_y, width, panel_h, argb(105, 0, 0, 0));
    fill_rect(0, panel_y, width, 2, argb(130, 255, 255, 255));

    const int left_x = 12;
    const int right_x = width / 2 + 6;
    int y = panel_y + 12;
    draw_text_left(left_x, y, format_text("STR %.1F/%.1F", hud.steering_angle, hud.desired_angle), 2, white, 222);
    draw_text_left(right_x, y, format_text("LANE %.2F/%.2F", hud.l_prob, hud.r_prob), 2, white, 222);
    y += 20;
    draw_text_left(left_x, y, format_text("ACC %.2F", hud.accel), 2, dim, 222);
    draw_text_left(right_x, y, format_text("LW %.1FM DP %.2F", hud.lane_width, hud.d_prob), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, hud.lead_status ? format_text("LEAD %.0FM %.1F", hud.lead_d, hud.lead_v) : "LEAD --", 2,
                   hud.lead_status ? white : dim, 222);
    draw_text_left(right_x, y, format_text("SR %.1F AO %.2F", hud.steer_ratio, hud.angle_offset_avg), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, format_text("CPU %.0F%% %.0FC", hud.cpu_usage, hud.cpu_temp), 2,
                   hud.cpu_temp >= 85.0f ? red : (hud.cpu_temp >= 75.0f ? orange : dim), 222);
    draw_text_left(right_x, y, format_text("SF %.2F MEM %.0F%%", hud.stiffness_factor, hud.memory_usage), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, format_text("BS %s/%s", hud.left_blindspot ? "L" : "-", hud.right_blindspot ? "R" : "-"), 2,
                   (hud.left_blindspot || hud.right_blindspot) ? red : dim, 222);
    draw_text_left(right_x, y, format_text("%s TCO %.2F", hud.laneless ? "LANELESS" : "LANE", -hud.total_camera_offset), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, tpms_text(), 2, tpms_available() ? dim : argb(130, 180, 185, 190), 222);
    draw_text_left(right_x, y, gps_text(), 2, hud.satellite_count > 0 ? dim : argb(130, 180, 185, 190), 222);
    y += 20;
    draw_text_left(left_x, y, format_text("M %.0F R%.2F/%.2F", hud.model_execution_ms, hud.road_edge_confs[0], hud.road_edge_confs[1]), 2, dim, 222);
    draw_text_left(right_x, y, format_text("L %.1F %.1F %.1F %.1F", hud.lane_line_probs[0], hud.lane_line_probs[1],
                                           hud.lane_line_probs[2], hud.lane_line_probs[3]), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, format_text("TQ %.1F RPM %.0F", hud.steering_torque, hud.engine_rpm), 2, dim, 222);
    draw_text_left(right_x, y, format_text("M %s AD %.2F", lateral_method_label(), hud.steer_actuator_delay), 2, dim, 222);
    y += 20;
    draw_text_left(left_x, y, format_text("%s%s%s", hud.brake_hold ? "HLD " : "",
                                          standstill_text().c_str(),
                                          hud.driver_acc ? " DRV" : ""), 2, dim, 222);
    draw_text_left(right_x, y, format_text("%s SC %.2F S %.0F", hud.longitudinal_control ? "LNG" : "LAT",
                                           hud.output_scale, hud.safety_speed), 2, dim, 222);

    if (alert_active) {
      const uint32_t alert_color = hud.alert_size >= 3 ? argb(210, 150, 0, 0) : argb(190, 170, 90, 0);
      const int alert_y = height - panel_h - 72;
      fill_rect(0, alert_y, width, 72, alert_color);
      draw_text_center(width / 2, alert_y + 12, hud.alert_text1.empty() ? "ALERT" : hud.alert_text1, 3, white, width - 24);
      draw_text_center(width / 2, alert_y + 44, hud.alert_text2.empty() ? hud.alert_type : hud.alert_text2, 2, white, width - 24);
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
  SubMaster sm;
  struct v4l2_drm_context ctx = {};
  struct display_plane *video_plane = nullptr;
  std::vector<display_buffer *> video_buffers;
  struct display_plane *overlay_plane = nullptr;
  struct display_buffer *overlay_buffer = nullptr;
  std::vector<display_buffer *> overlay_buffers;
  std::vector<K230Rect> overlay_dirty_rects;
  K230Rect current_overlay_dirty;
  K230Mat3 view_from_calib = {};
  K230HudState hud;
  uint64_t last_model_frame = 0;
  uint64_t overlay_frame_counter = 0;
  size_t video_buffer_index = 0;
  size_t overlay_buffer_index = 0;
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
