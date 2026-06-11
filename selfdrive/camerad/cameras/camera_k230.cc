#include "selfdrive/camerad/cameras/camera_common.h"

#include <linux/videodev2.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/k230_vvcam.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"

#include "v4l2-drm.h"

ExitHandler do_exit;

namespace {

constexpr unsigned K230_DEFAULT_WIDTH = 512;
constexpr unsigned K230_DEFAULT_HEIGHT = 288;
constexpr unsigned K230_DEFAULT_SENSOR_WIDTH = 1920;
constexpr unsigned K230_DEFAULT_SENSOR_HEIGHT = 1080;
constexpr int K230_DEFAULT_TIMEOUT_MS = 1000;
constexpr int K230_DEFAULT_VIPC_BUFFERS = 40;
constexpr int K230_DEFAULT_V4L2_BUFFERS = 5;

struct K230CameraConfig {
  int device = -1;
  unsigned width = K230_DEFAULT_WIDTH;
  unsigned height = K230_DEFAULT_HEIGHT;
  unsigned crop_x = 0;
  unsigned crop_y = 0;
  unsigned crop_width = 0;
  unsigned crop_height = 0;
  int timeout_ms = K230_DEFAULT_TIMEOUT_MS;
  int vipc_buffers = K230_DEFAULT_VIPC_BUFFERS;
  int v4l2_buffers = K230_DEFAULT_V4L2_BUFFERS;
};

K230CameraConfig read_config() {
  K230CameraConfig cfg = {};

  if (!k230_vvcam::wait_for_ready()) {
    throw std::runtime_error("K230 vvcam daemon/video nodes not ready");
  }
  const int video00 = k230_vvcam::detect_vvcam_video00();
  if (video00 < 0) {
    throw std::runtime_error("K230 vvcam-video.0.0 not found");
  }
  cfg.device = video00 + 1;

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

class K230V4l2DrmCapture {
public:
  explicit K230V4l2DrmCapture(const K230CameraConfig &cfg) {
    v4l2_drm_default_context(&ctx);
    ctx.device = cfg.device;
    ctx.width = cfg.width;
    ctx.height = cfg.height;
    ctx.video_format = V4L2_PIX_FMT_NV12;
    ctx.crop_size.offset_x = cfg.crop_x;
    ctx.crop_size.offset_y = cfg.crop_y;
    ctx.crop_size.width = cfg.crop_width;
    ctx.crop_size.height = cfg.crop_height;
    ctx.buffer_num = static_cast<unsigned>(cfg.v4l2_buffers);

    LOGW("K230 camerad opening /dev/video%d %ux%u NV12 crop=%ux%u+%u+%u",
         cfg.device, cfg.width, cfg.height, cfg.crop_width, cfg.crop_height, cfg.crop_x, cfg.crop_y);

    k230_vvcam::SetupLock setup_lock;
    if (!setup_lock.locked()) {
      LOGW("K230 camerad could not lock vvcam setup");
    }

    if (v4l2_drm_setup(&ctx, 1) != 0) {
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
  }

  ~K230V4l2DrmCapture() {
    if (setup) {
      v4l2_drm_stop(&ctx);
    }
  }

  v4l2_drm_context ctx = {};
  bool setup = false;
};

void run_k230_camerad() {
  const K230CameraConfig cfg = read_config();
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
