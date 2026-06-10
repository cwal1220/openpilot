#include <linux/videodev2.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
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

struct K230Mat3 {
  float v[9] = {};
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
      : cfg(init_cfg), display(init_display), sm({"modelV2", "liveCalibration"}) {
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
      rotate_nv12_90(src, video_buffers[video_buffer_index]);
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
    if (overlay_buffer != nullptr) {
      display_free_buffer(overlay_buffer);
      overlay_buffer = nullptr;
    }
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

  void rotate_nv12_90(const uint8_t *src, display_buffer *dst) {
    const int src_w = static_cast<int>(cfg.width);
    const int src_h = static_cast<int>(cfg.height);
    const int dst_h = static_cast<int>(dst->height);
    auto *dst_y = static_cast<uint8_t *>(dst->map);
    auto *dst_uv = dst_y + dst->stride * dst_h;
    const uint8_t *src_y = src;
    const uint8_t *src_uv = src + src_w * src_h;

    for (int y = 0; y < src_h; ++y) {
      for (int x = 0; x < src_w; ++x) {
        const int dx = src_h - 1 - y;
        const int dy = x;
        dst_y[dy * dst->stride + dx] = src_y[y * src_w + x];
      }
    }

    const int src_ch = src_h / 2;
    const int src_cw = src_w / 2;
    for (int cy = 0; cy < src_ch; ++cy) {
      for (int cx = 0; cx < src_cw; ++cx) {
        const int dcx = src_ch - 1 - cy;
        const int dcy = cx;
        const uint8_t *s = src_uv + cy * src_w + cx * 2;
        uint8_t *d = dst_uv + dcy * dst->stride + dcx * 2;
        d[0] = s[0];
        d[1] = s[1];
      }
    }
  }

  void setup_overlay() {
    overlay_plane = display_get_plane(display, DRM_FORMAT_ARGB8888);
    if (overlay_plane == nullptr) {
      LOGE("K230 previewd could not allocate ARGB overlay plane");
      return;
    }

    overlay_buffer = display_allocate_buffer(overlay_plane, display->width, display->height);
    if (overlay_buffer == nullptr) {
      LOGE("K230 previewd could not allocate ARGB overlay buffer");
      display_free_plane(overlay_plane);
      overlay_plane = nullptr;
      return;
    }

    std::memset(overlay_buffer->map, 0, overlay_buffer->size);
    overlay_scratch.resize(overlay_buffer->size);
  }

  bool update_model_state() {
    bool changed = false;
    sm.update(0);
    if (sm.updated("liveCalibration")) {
      auto rpy = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
      if (rpy.size() >= 3 && std::isfinite(rpy[0]) && std::isfinite(rpy[1]) && std::isfinite(rpy[2])) {
        view_from_calib = view_from_calib_from_rpy(rpy[0], rpy[1], rpy[2]);
        changed = true;
      }
    }
    return changed;
  }

  void update_overlay() {
    const bool calib_changed = update_model_state();
    if (overlay_buffer == nullptr) return;

    const uint64_t model_frame = sm.rcv_frame("modelV2");
    if (!first_overlay && !calib_changed && model_frame == last_model_frame) {
      return;
    }

    std::memset(overlay_scratch.data(), 0, overlay_scratch.size());
    if (model_frame > 0) {
      draw_model(sm["modelV2"].getModelV2());
    }
    first_overlay = false;
    last_model_frame = model_frame;

    std::memcpy(overlay_buffer->map, overlay_scratch.data(), overlay_buffer->size);
    if (display_update_buffer(overlay_buffer, 0, 0) != 0) {
      LOGE("K230 previewd overlay update failed");
    }
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

  void fill_poly(const K230Poly &poly, uint32_t color) {
    if (poly.cnt < 3) return;

    auto *pixels = reinterpret_cast<uint32_t *>(overlay_scratch.data());
    const int width = static_cast<int>(overlay_buffer->stride / 4);
    const int height = static_cast<int>(overlay_buffer->height);

    float min_y = poly.points[0].y;
    float max_y = poly.points[0].y;
    for (int i = 1; i < poly.cnt; ++i) {
      min_y = std::min(min_y, poly.points[i].y);
      max_y = std::max(max_y, poly.points[i].y);
    }

    const int y0 = std::max(0, static_cast<int>(std::floor(min_y)));
    const int y1 = std::min(height - 1, static_cast<int>(std::ceil(max_y)));
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
  std::vector<uint8_t> overlay_scratch;
  K230Mat3 view_from_calib = {};
  uint64_t last_model_frame = 0;
  size_t video_buffer_index = 0;
  bool setup = false;
  bool first_overlay = true;
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
