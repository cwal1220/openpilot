#include "selfdrive/camerad/cameras/camera_common.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"

#include "v4l2-drm.h"

ExitHandler do_exit;

namespace {

constexpr unsigned K230_DEFAULT_WIDTH = 512;
constexpr unsigned K230_DEFAULT_HEIGHT = 256;
constexpr unsigned K230_DEFAULT_SENSOR_WIDTH = 1920;
constexpr unsigned K230_DEFAULT_SENSOR_HEIGHT = 1080;
constexpr int K230_DEFAULT_DEVICE = 2;
constexpr int K230_DEFAULT_TIMEOUT_MS = 1000;
constexpr int K230_DEFAULT_VIPC_BUFFERS = 40;
constexpr int K230_DEFAULT_V4L2_BUFFERS = 5;
constexpr int K230_OVERLAY_VERTICES = TRAJECTORY_SIZE * 2;

constexpr float K230_OV5647_FX = 1625.7416788144435f;
constexpr float K230_OV5647_FY = 1585.9830269782024f;
constexpr float K230_OV5647_CX = 946.13450988811394f;
constexpr float K230_OV5647_CY = 537.34063862123787f;

struct K230CameraConfig {
  int device = K230_DEFAULT_DEVICE;
  unsigned width = K230_DEFAULT_WIDTH;
  unsigned height = K230_DEFAULT_HEIGHT;
  unsigned crop_x = 0;
  unsigned crop_y = 0;
  unsigned crop_width = K230_DEFAULT_SENSOR_WIDTH;
  unsigned crop_height = K230_DEFAULT_SENSOR_HEIGHT;
  int timeout_ms = K230_DEFAULT_TIMEOUT_MS;
  int vipc_buffers = K230_DEFAULT_VIPC_BUFFERS;
  int v4l2_buffers = K230_DEFAULT_V4L2_BUFFERS;
  bool preview = false;
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

K230Point rotate_overlay_point(float x, float y, unsigned logical_width, unsigned logical_height,
                               unsigned overlay_width, unsigned overlay_height,
                               enum drm_rotation rotation) {
  (void)logical_width;
  if (rotation == rotation_90) {
    return {static_cast<float>(logical_height) - y, x};
  } else if (rotation == rotation_180) {
    return {static_cast<float>(logical_width) - x,
            static_cast<float>(logical_height) - y};
  } else if (rotation == rotation_270) {
    return {y, static_cast<float>(logical_width) - x};
  }
  return {std::clamp(x, 0.0f, static_cast<float>(overlay_width)),
          std::clamp(y, 0.0f, static_cast<float>(overlay_height))};
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

int parse_video_device(const std::string &value) {
  if (value.empty()) return K230_DEFAULT_DEVICE;

  const std::string prefix = "/dev/video";
  const std::string digits = value.rfind(prefix, 0) == 0 ? value.substr(prefix.size()) : value;
  char *end = nullptr;
  long device = std::strtol(digits.c_str(), &end, 10);
  if (end == digits.c_str() || *end != '\0' || device < 0 || device > 63) {
    throw std::runtime_error("bad K230_CAM_DEVICE: " + value);
  }
  return static_cast<int>(device);
}

int detect_vvcam_video00() {
  for (int i = 0; i < 20; ++i) {
    const std::string dev_path = util::string_format("/dev/video%d", i);
    int fd = HANDLE_EINTR(open(dev_path.c_str(), O_RDONLY | O_CLOEXEC));
    if (fd < 0) continue;

    v4l2_capability cap = {};
    const int ret = HANDLE_EINTR(ioctl(fd, VIDIOC_QUERYCAP, &cap));
    close(fd);
    if (ret != 0) continue;

    if (std::strcmp(reinterpret_cast<const char *>(cap.card), "vvcam-video.0.0") == 0) {
      return i;
    }
  }
  return -1;
}

K230CameraConfig read_config() {
  K230CameraConfig cfg = {};
  cfg.preview = util::getenv("K230_PREVIEW", 0) != 0;

  const std::string env_device = util::getenv("K230_CAM_DEVICE");
  if (!env_device.empty()) {
    cfg.device = parse_video_device(env_device);
  } else {
    const int video00 = detect_vvcam_video00();
    cfg.device = video00 >= 0 ? video00 + (cfg.preview ? 0 : 1) : K230_DEFAULT_DEVICE;
  }

  if ((cfg.width & 1) || (cfg.height & 1) || (cfg.crop_width & 1) || (cfg.crop_height & 1)) {
    throw std::runtime_error("K230 camera dimensions must be even");
  }
  return cfg;
}

void nv12_to_i420(const uint8_t *nv12, VisionBuf *dst) {
  const size_t width = dst->width;
  const size_t height = dst->height;
  const size_t y_size = width * height;
  const size_t half_width = width / 2;
  const size_t half_height = height / 2;

  std::memcpy(dst->y, nv12, y_size);
  const uint8_t *uv = nv12 + y_size;

  for (size_t y = 0; y < half_height; ++y) {
    const uint8_t *uv_row = uv + y * width;
    uint8_t *u_row = dst->u + y * half_width;
    uint8_t *v_row = dst->v + y * half_width;
    for (size_t x = 0; x < half_width; ++x) {
      u_row[x] = uv_row[x * 2];
      v_row[x] = uv_row[x * 2 + 1];
    }
  }
}

class K230PreviewRuntime;
K230PreviewRuntime *preview_runtime = nullptr;

class K230PreviewRuntime {
public:
  K230PreviewRuntime(const K230CameraConfig &init_cfg, struct display *init_display, enum drm_rotation init_rotation)
      : cfg(init_cfg), display(init_display), rotation(init_rotation), frame_bytes(static_cast<size_t>(init_cfg.width) * init_cfg.height * 3 / 2),
        vipc_server("camerad"), pm({"roadCameraState"}), sm({"modelV2", "liveCalibration"}) {
    view_from_calib = view_from_calib_from_rpy(0.0f, 0.0f, 0.0f);
  }

  int run() {
    vipc_server.create_buffers(VISION_STREAM_ROAD, cfg.vipc_buffers, false, cfg.width, cfg.height);
    vipc_server.start_listener();

    v4l2_drm_default_context(&ctx);
    ctx.device = cfg.device;
    ctx.display = true;
    ctx.width = cfg.width;
    ctx.height = cfg.height;
    ctx.video_format = V4L2_PIX_FMT_NV12;
    ctx.display_format = 0;
    ctx.drm_rotation = rotation;
    ctx.buffer_num = static_cast<unsigned>(cfg.v4l2_buffers);

    LOGW("K230 preview opening /dev/video%d %ux%u NV12 rotation=%d",
         cfg.device, cfg.width, cfg.height, rotation);

    if (v4l2_drm_setup(&ctx, 1, &display) != 0) {
      throw std::runtime_error(util::string_format("v4l2_drm_setup preview failed for /dev/video%d errno=%d (%s)",
                                                   cfg.device, errno, std::strerror(errno)));
    }
    setup = true;

    setup_overlay();

    preview_runtime = this;
    const int ret = v4l2_drm_run(&ctx, 1, &K230PreviewRuntime::handle_frame);
    preview_runtime = nullptr;
    return ret;
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
    if (display != nullptr) {
      display_exit(display);
      display = nullptr;
    }
  }

private:
  static int handle_frame(struct v4l2_drm_context *ctx, bool displayed) {
    return preview_runtime != nullptr ? preview_runtime->on_frame(ctx, displayed) : 0;
  }

  void setup_overlay() {
    overlay_plane = display_get_plane(display, DRM_FORMAT_ARGB8888);
    if (overlay_plane == nullptr) {
      LOGE("K230 preview could not allocate ARGB overlay plane");
      return;
    }

    overlay_buffer = display_allocate_buffer(overlay_plane, display->width, display->height);
    if (overlay_buffer == nullptr) {
      LOGE("K230 preview could not allocate ARGB overlay buffer");
      display_free_plane(overlay_plane);
      overlay_plane = nullptr;
      return;
    }

    std::memset(overlay_buffer->map, 0, overlay_buffer->size);
    overlay_scratch.resize(overlay_buffer->size);
    if (display_commit_buffer(overlay_buffer, 0, 0) != 0) {
      LOGE("K230 preview initial overlay commit failed");
    }
  }

  int on_frame(struct v4l2_drm_context *contexts, bool displayed) {
    auto *context = &contexts[0];
    if (context->flag_dqbuf && context->frame_count != last_sent_capture_count &&
        context->vbuffer.index < context->buffer_num) {
      publish_frame(context);
      last_sent_capture_count = context->frame_count;
    }

    update_overlay(context, displayed);
    return do_exit ? 'q' : 0;
  }

  void publish_frame(struct v4l2_drm_context *context) {
    const uint64_t ts = nanos_since_boot();
    const auto *src = static_cast<const uint8_t *>(context->buffers[context->vbuffer.index].mmap);
    if (src == nullptr) {
      LOGE("K230 preview got null V4L2 buffer");
      return;
    }

    VisionBuf *buf = vipc_server.get_buffer(VISION_STREAM_ROAD);
    if (buf->len != frame_bytes) {
      LOGE("K230 preview VisionIPC buffer size mismatch");
      return;
    }

    nv12_to_i420(src, buf);

    VisionIpcBufExtra extra = {
      .frame_id = frame_id,
      .timestamp_sof = ts,
      .timestamp_eof = ts,
    };
    vipc_server.send(buf, &extra, false);

    MessageBuilder msg;
    auto framed = msg.initEvent().initRoadCameraState();
    framed.setFrameId(frame_id);
    framed.setTimestampSof(ts);
    framed.setTimestampEof(ts);
    framed.setProcessingTime(0.0f);
    pm.send("roadCameraState", msg);

    ++frame_id;
  }

  void update_model_state() {
    sm.update(0);
    if (sm.updated("liveCalibration")) {
      auto rpy = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
      if (rpy.size() >= 3 && std::isfinite(rpy[0]) && std::isfinite(rpy[1]) && std::isfinite(rpy[2])) {
        view_from_calib = view_from_calib_from_rpy(rpy[0], rpy[1], rpy[2]);
      }
    }
  }

  void update_overlay(struct v4l2_drm_context *context, bool displayed) {
    update_model_state();
    if (!displayed || overlay_buffer == nullptr) return;

    display_buffer *current_preview_buffer = nullptr;
    if (context->buffer_hold[context->wp] >= 0) {
      current_preview_buffer = context->display_buffers[context->buffer_hold[context->wp]];
    }
    const bool preview_updated = current_preview_buffer != nullptr && current_preview_buffer != last_preview_buffer;

    if (!preview_updated && !first_overlay) {
      return;
    }

    std::memset(overlay_scratch.data(), 0, overlay_scratch.size());
    if (sm.rcv_frame("modelV2") > 0) {
      draw_model(sm["modelV2"].getModelV2());
    }
    first_overlay = false;

    std::memcpy(overlay_buffer->map, overlay_scratch.data(), overlay_buffer->size);
    if (display_update_buffer(overlay_buffer, 0, 0) != 0) {
      LOGE("K230 preview overlay update failed");
    }
    last_preview_buffer = current_preview_buffer;
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
    const float sx = static_cast<float>(cfg.width) / K230_DEFAULT_SENSOR_WIDTH;
    const float sy = static_cast<float>(cfg.height) / K230_DEFAULT_SENSOR_HEIGHT;
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

    *out = rotate_overlay_point(px, py, cfg.width, cfg.height, overlay_buffer->width, overlay_buffer->height, rotation);
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
          uint32_t &dst = pixels[y * width + x];
          dst = color;
        }
      }
    }
  }

  K230CameraConfig cfg;
  struct display *display = nullptr;
  enum drm_rotation rotation = rotation_0;
  size_t frame_bytes = 0;
  VisionIpcServer vipc_server;
  PubMaster pm;
  SubMaster sm;
  struct v4l2_drm_context ctx = {};
  struct display_plane *overlay_plane = nullptr;
  struct display_buffer *overlay_buffer = nullptr;
  struct display_buffer *last_preview_buffer = nullptr;
  std::vector<uint8_t> overlay_scratch;
  K230Mat3 view_from_calib = {};
  uint32_t frame_id = 0;
  unsigned last_sent_capture_count = 0;
  bool setup = false;
  bool first_overlay = true;
};

class K230V4l2DrmCapture {
public:
  explicit K230V4l2DrmCapture(const K230CameraConfig &cfg) {
    v4l2_drm_default_context(&ctx);
    ctx.device = cfg.device;
    ctx.display = false;
    ctx.width = cfg.width;
    ctx.height = cfg.height;
    ctx.video_format = V4L2_PIX_FMT_NV12;
    ctx.crop_size.crop_en = 1;
    ctx.crop_size.offset_x = cfg.crop_x;
    ctx.crop_size.offset_y = cfg.crop_y;
    ctx.crop_size.width = cfg.crop_width;
    ctx.crop_size.height = cfg.crop_height;
    ctx.buffer_num = static_cast<unsigned>(cfg.v4l2_buffers);

    LOGW("K230 camerad opening /dev/video%d %ux%u NV12 crop=%ux%u+%u+%u",
         cfg.device, cfg.width, cfg.height, cfg.crop_width, cfg.crop_height, cfg.crop_x, cfg.crop_y);

    if (v4l2_drm_setup(&ctx, 1, nullptr) != 0) {
      throw std::runtime_error(util::string_format("v4l2_drm_setup failed for /dev/video%d errno=%d (%s)",
                                                   cfg.device, errno, std::strerror(errno)));
    }
    setup = true;
    if (v4l2_drm_start(&ctx) != 0) {
      const int start_errno = errno;
      v4l2_drm_stop(&ctx);
      setup = false;
      errno = start_errno;
      throw std::runtime_error(util::string_format("v4l2_drm_start failed for /dev/video%d errno=%d (%s)",
                                                   cfg.device, errno, std::strerror(errno)));
    }
    started = true;
  }

  ~K230V4l2DrmCapture() {
    if (setup) {
      v4l2_drm_stop(&ctx);
    }
  }

  v4l2_drm_context ctx = {};
  bool setup = false;
  bool started = false;
};

void run_k230_camerad() {
  const K230CameraConfig cfg = read_config();
  if (cfg.preview) {
    struct display *display = display_init(0);
    if (display == nullptr) {
      throw std::runtime_error("display_init failed for K230 preview");
    }

    K230CameraConfig preview_cfg = cfg;
    if (display->width < display->height) {
      preview_cfg.width = display->height;
      preview_cfg.height = display->width;
    } else {
      preview_cfg.width = display->width;
      preview_cfg.height = display->height;
    }

    enum drm_rotation rotation = display->width < display->height ? rotation_90 : rotation_0;

    K230PreviewRuntime preview(preview_cfg, display, rotation);
    preview.run();
    return;
  }

  const size_t frame_bytes = static_cast<size_t>(cfg.width) * cfg.height * 3 / 2;

  VisionIpcServer vipc_server("camerad");
  vipc_server.create_buffers(VISION_STREAM_ROAD, cfg.vipc_buffers, false, cfg.width, cfg.height);
  vipc_server.start_listener();

  PubMaster pm({"roadCameraState"});
  K230V4l2DrmCapture capture(cfg);

  uint32_t frame_id = 0;
  uint32_t error_count = 0;
  while (!do_exit) {
    const uint64_t ts_sof = nanos_since_boot();
    if (v4l2_drm_dump(&capture.ctx, cfg.timeout_ms) != 0) {
      ++error_count;
      LOGE("K230 camerad v4l2_drm_dump failed errno=%d (%s), errors=%u", errno, std::strerror(errno), error_count);
      util::sleep_for(50);
      continue;
    }

    const uint64_t ts_eof = nanos_since_boot();
    const auto *src = static_cast<const uint8_t *>(capture.ctx.buffers[capture.ctx.vbuffer.index].mmap);
    if (src == nullptr) {
      v4l2_drm_dump_release(&capture.ctx);
      ++error_count;
      LOGE("K230 camerad got null v4l2 buffer, errors=%u", error_count);
      continue;
    }

    VisionBuf *buf = vipc_server.get_buffer(VISION_STREAM_ROAD);
    if (buf->len != frame_bytes) {
      v4l2_drm_dump_release(&capture.ctx);
      throw std::runtime_error("K230 VisionIPC buffer size mismatch");
    }

    nv12_to_i420(src, buf);
    v4l2_drm_dump_release(&capture.ctx);

    VisionIpcBufExtra extra = {
      .frame_id = frame_id,
      .timestamp_sof = ts_sof,
      .timestamp_eof = ts_eof,
    };
    vipc_server.send(buf, &extra, false);

    MessageBuilder msg;
    auto framed = msg.initEvent().initRoadCameraState();
    framed.setFrameId(frame_id);
    framed.setTimestampSof(ts_sof);
    framed.setTimestampEof(ts_eof);
    framed.setProcessingTime(static_cast<float>((ts_eof - ts_sof) * 1e-9));
    pm.send("roadCameraState", msg);

    ++frame_id;
  }
}

}  // namespace

void camerad_thread() {
  try {
    run_k230_camerad();
  } catch (const std::exception &e) {
    LOGE("K230 camerad error: %s", e.what());
    std::exit(1);
  }
}
