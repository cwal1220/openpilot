#include "selfdrive/camerad/cameras/camera_webcam.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>
#include <thread>

#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"

extern ExitHandler do_exit;

namespace {

int get_env_int(const char *name, int default_value) {
  return util::getenv(name, default_value);
}

std::string get_env_str(const char *name, const std::string &default_value = "") {
  const char *value = getenv(name);
  return (value != nullptr && value[0] != '\0') ? std::string(value) : default_value;
}

int parse_fourcc(const std::string &fourcc) {
  if (fourcc.size() != 4) return 0;
  return cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
}

std::string fourcc_to_string(int fourcc) {
  if (fourcc == 0) return "unset";
  char code[5] = {
    static_cast<char>(fourcc & 0xFF),
    static_cast<char>((fourcc >> 8) & 0xFF),
    static_cast<char>((fourcc >> 16) & 0xFF),
    static_cast<char>((fourcc >> 24) & 0xFF),
    '\0',
  };
  return std::string(code);
}

bool driver_monitoring_enabled() {
  return util::getenv("DISABLE_DRIVER_MONITORING", 0) == 0;
}

void fill_metadata(FrameMetadata *meta, uint32_t frame_id, size_t frame_len) {
  const uint64_t ts = nanos_since_boot();
  *meta = {};
  meta->frame_id = frame_id;
  meta->frame_length = static_cast<unsigned int>(frame_len);
  meta->timestamp_sof = ts;
  meta->timestamp_eof = ts;
  meta->integ_lines = 0;
  meta->high_conversion_gain = false;
  meta->gain = 1.0f;
  meta->measured_grey_fraction = 0.0f;
  meta->target_grey_fraction = 0.0f;
  meta->lens_pos = 0;
  meta->lens_err = 0.0f;
  meta->lens_true_pos = 0.0f;
  meta->processing_time = 0.0f;
}

void log_capture_settings(const char *label, const CameraState &s, cv::VideoCapture &cap) {
  const int actual_fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
  const double actual_fps = cap.get(cv::CAP_PROP_FPS);
  const int actual_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  LOGW("%s webcam device=%s requested=%dx%d@%d fourcc=%s actual=%dx%d@%.2f fourcc=%s",
       label, s.device_path.c_str(), s.ci.frame_width, s.ci.frame_height, s.fps,
       get_env_str(s.camera_num == 0 ? "ROADCAM_FOURCC" : "DRIVERCAM_FOURCC", "auto").c_str(),
       actual_width, actual_height, actual_fps, fourcc_to_string(actual_fourcc).c_str());
}

cv::VideoCapture open_camera(const CameraState &s) {
  cv::VideoCapture cap;
  if (!s.device_path.empty()) {
    cap.open(s.device_path, cv::CAP_V4L2);
  } else {
    cap.open(s.camera_index, cv::CAP_V4L2);
  }

  if (!cap.isOpened()) {
    if (!s.device_path.empty()) {
      LOGE("failed to open webcam device %s", s.device_path.c_str());
    } else {
      LOGE("failed to open webcam index %d", s.camera_index);
    }
  }
  return cap;
}

void apply_common_settings(cv::VideoCapture &cap, const CameraState &s, int width, int height) {
  const std::string fourcc_name = get_env_str(s.camera_num == 0 ? "ROADCAM_FOURCC" : "DRIVERCAM_FOURCC");
  const int fourcc = parse_fourcc(fourcc_name);
  if (fourcc != 0) {
    cap.set(cv::CAP_PROP_FOURCC, fourcc);
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, s.fps);
  cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
  cap.set(cv::CAP_PROP_CONVERT_RGB, 1);
}

CameraInfo probe_camera(CameraState *s, int desired_width, int desired_height) {
  cv::VideoCapture cap = open_camera(*s);
  if (!cap.isOpened()) {
    assert(false);
  }

  apply_common_settings(cap, *s, desired_width, desired_height);

  int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  double actual_fps = cap.get(cv::CAP_PROP_FPS);
  if (width <= 0) width = desired_width;
  if (height <= 0) height = desired_height;
  if (actual_fps > 0.0) {
    s->fps = std::max(1, static_cast<int>(std::lround(actual_fps)));
  }
  CameraInfo ci = {};
  ci.frame_width = width;
  ci.frame_height = height;
  ci.frame_stride = width * 3;
  ci.bayer = false;
  ci.bayer_flip = 0;
  ci.hdr = false;
  return ci;
}

void configure_capture(cv::VideoCapture &cap, const CameraState &s) {
  apply_common_settings(cap, s, s.ci.frame_width, s.ci.frame_height);
  log_capture_settings(s.camera_num == 0 ? "road" : "driver", s, cap);
}

void queue_frame(CameraState *s, const cv::Mat &input_frame, uint32_t *frame_id, size_t *buf_idx) {
  if (s->buf.pending_frames() >= FRAME_BUF_COUNT - 1) {
    return;
  }

  cv::Mat working = input_frame;
  cv::Mat resized;
  cv::Mat bgr;
  const size_t frame_len = static_cast<size_t>(s->ci.frame_height) * s->ci.frame_stride;

  if (working.cols != s->ci.frame_width || working.rows != s->ci.frame_height) {
    cv::resize(working, resized, cv::Size(s->ci.frame_width, s->ci.frame_height), 0, 0, cv::INTER_LINEAR);
    working = resized;
  }

  if (working.channels() == 4) {
    cv::cvtColor(working, bgr, cv::COLOR_BGRA2BGR);
  } else if (working.channels() == 1) {
    cv::cvtColor(working, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = working;
  }

  if (!bgr.isContinuous()) {
    bgr = bgr.clone();
  }

  auto &buf = s->buf.camera_bufs[*buf_idx];
  std::memcpy(buf.addr, bgr.data, std::min(frame_len, bgr.total() * bgr.elemSize()));
  fill_metadata(&s->buf.camera_bufs_metadata[*buf_idx], (*frame_id)++, frame_len);
  if (*frame_id == 1) {
    LOGW("%s webcam queued first frame len=%zu pending=%zu",
         s->camera_num == 0 ? "road" : "driver", frame_len, s->buf.pending_frames());
  }
  if (s->ci.bayer) {
    buf.sync(VISIONBUF_SYNC_TO_DEVICE);
  }
  s->buf.queue(*buf_idx);
  *buf_idx = (*buf_idx + 1) % FRAME_BUF_COUNT;
}

void capture_thread(CameraState *s) {
  util::set_thread_name(s->camera_num == 0 ? "webcam_road_capture" : "webcam_driver_capture");

  assert(s->cap != nullptr);
  cv::VideoCapture &cap = *s->cap;

  uint32_t frame_id = 0;
  size_t buf_idx = 0;
  cv::Mat frame;
  int consecutive_failures = 0;

  while (!do_exit) {
    if (s->buf.pending_frames() >= FRAME_BUF_COUNT - 1) {
      util::sleep_for(2);
      continue;
    }

    if (!cap.read(frame) || frame.empty()) {
      consecutive_failures++;
      if (consecutive_failures == 1 || (consecutive_failures % 50) == 0) {
        LOGW("%s webcam read failed (count=%d pending=%zu)",
             s->camera_num == 0 ? "road" : "driver", consecutive_failures, s->buf.pending_frames());
      }
      util::sleep_for(10);
      continue;
    }

    if (frame_id == 0) {
      LOGW("%s webcam first frame shape=%dx%d channels=%d",
           s->camera_num == 0 ? "road" : "driver", frame.cols, frame.rows, frame.channels());
    }
    consecutive_failures = 0;
    queue_frame(s, frame, &frame_id, &buf_idx);
  }
}

void shared_capture_thread(MultiCameraState *s) {
  util::set_thread_name("webcam_shared_capture");

  assert(s->road_cam.cap != nullptr);
  cv::VideoCapture &cap = *s->road_cam.cap;

  uint32_t road_frame_id = 0;
  uint32_t driver_frame_id = 0;
  size_t road_buf_idx = 0;
  size_t driver_buf_idx = 0;
  cv::Mat frame;
  int consecutive_failures = 0;

  while (!do_exit) {
    if (s->road_cam.buf.pending_frames() >= FRAME_BUF_COUNT - 1 &&
        s->driver_cam.buf.pending_frames() >= FRAME_BUF_COUNT - 1) {
      util::sleep_for(2);
      continue;
    }

    if (!cap.read(frame) || frame.empty()) {
      consecutive_failures++;
      if (consecutive_failures == 1 || (consecutive_failures % 50) == 0) {
        LOGW("shared webcam read failed (count=%d road_pending=%zu driver_pending=%zu)",
             consecutive_failures, s->road_cam.buf.pending_frames(), s->driver_cam.buf.pending_frames());
      }
      util::sleep_for(10);
      continue;
    }

    if (road_frame_id == 0) {
      LOGW("shared webcam first frame shape=%dx%d channels=%d", frame.cols, frame.rows, frame.channels());
    }
    consecutive_failures = 0;
    queue_frame(&s->road_cam, frame, &road_frame_id, &road_buf_idx);
    queue_frame(&s->driver_cam, frame, &driver_frame_id, &driver_buf_idx);
  }
}

void process_road_camera(MultiCameraState *s, CameraState *c, int cnt) {
  const CameraBuf *b = &c->buf;
  MessageBuilder msg;
  auto framed = msg.initEvent().initRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data);
  if (env_send_road) {
    framed.setImage(get_frame_image(b));
  }
  framed.setTransform(b->yuv_transform.v);
  s->pm->send("roadCameraState", msg);
}

void process_driver_camera(MultiCameraState *s, CameraState *c, int cnt) {
  common_process_driver_camera(s, c, cnt);
}

}  // namespace

void camera_autoexposure(CameraState *s, float grey_frac) {}

static void init_camera(CameraState *s, int camera_num, int camera_index, int fps, int desired_width, int desired_height) {
  s->camera_num = camera_num;
  s->camera_id = camera_num;
  s->camera_index = camera_index;
  s->device_path = get_env_str(camera_num == 0 ? "ROADCAM_DEV" : "DRIVERCAM_DEV",
                               util::string_format("/dev/video%d", camera_index));
  s->fps = fps;
  s->ci = probe_camera(s, desired_width, desired_height);
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  const bool dm_enabled = driver_monitoring_enabled();
  const int road_index = get_env_int("ROADCAM_ID", 0);
  const int road_width = get_env_int("ROADCAM_WIDTH", 1280);
  const int road_height = get_env_int("ROADCAM_HEIGHT", 720);
  const int road_fps = get_env_int("ROADCAM_FPS", 20);

  init_camera(&s->road_cam, 0, road_index, road_fps, road_width, road_height);
  s->road_cam.buf.init(device_id, ctx, &s->road_cam, v, FRAME_BUF_COUNT, VISION_STREAM_RGB_ROAD, VISION_STREAM_ROAD);

  if (dm_enabled) {
    const int driver_index = get_env_int("DRIVERCAM_ID", road_index);
    const int driver_width = get_env_int("DRIVERCAM_WIDTH", 1280);
    const int driver_height = get_env_int("DRIVERCAM_HEIGHT", 720);
    const int driver_fps = get_env_int("DRIVERCAM_FPS", 10);
    init_camera(&s->driver_cam, 1, driver_index, driver_fps, driver_width, driver_height);
    s->driver_cam.buf.init(device_id, ctx, &s->driver_cam, v, FRAME_BUF_COUNT, VISION_STREAM_RGB_DRIVER, VISION_STREAM_DRIVER);
    s->sm = new SubMaster({"driverState"});
    s->pm = new PubMaster({"roadCameraState", "driverCameraState", "thumbnail"});
  } else {
    s->pm = new PubMaster({"roadCameraState", "thumbnail"});
  }
}

void cameras_open(MultiCameraState *s) {
  const bool dm_enabled = driver_monitoring_enabled();
  s->road_cam.cap = std::make_unique<cv::VideoCapture>(open_camera(s->road_cam));
  if (!s->road_cam.cap->isOpened()) {
    LOGE("failed to open road webcam device %s", s->road_cam.device_path.c_str());
    assert(false);
  }

  if (!dm_enabled) {
    configure_capture(*s->road_cam.cap, s->road_cam);
    return;
  }

  if (s->driver_cam.camera_index == s->road_cam.camera_index) {
    const int capture_fps = std::max(s->road_cam.fps, s->driver_cam.fps);
    const int capture_width = std::max(s->road_cam.ci.frame_width, s->driver_cam.ci.frame_width);
    const int capture_height = std::max(s->road_cam.ci.frame_height, s->driver_cam.ci.frame_height);
    s->road_cam.fps = capture_fps;
    apply_common_settings(*s->road_cam.cap, s->road_cam, capture_width, capture_height);
    LOGW("sharing webcam index %d for road and driver streams", s->road_cam.camera_index);
    return;
  }

  s->driver_cam.cap = std::make_unique<cv::VideoCapture>(open_camera(s->driver_cam));
  if (!s->driver_cam.cap->isOpened()) {
    LOGE("failed to open driver webcam device %s", s->driver_cam.device_path.c_str());
    assert(false);
  }

  configure_capture(*s->road_cam.cap, s->road_cam);
  configure_capture(*s->driver_cam.cap, s->driver_cam);
}

void cameras_close(MultiCameraState *s) {
  if (s->road_cam.cap) s->road_cam.cap->release();
  if (s->driver_cam.cap) s->driver_cam.cap->release();
  if (s->sm != nullptr) delete s->sm;
  if (s->pm != nullptr) delete s->pm;
}

void cameras_run(MultiCameraState *s) {
  const bool dm_enabled = driver_monitoring_enabled();
  std::vector<std::thread> threads;
  if (!dm_enabled) {
    threads.emplace_back(capture_thread, &s->road_cam);
  } else if (s->road_cam.camera_index == s->driver_cam.camera_index) {
    threads.emplace_back(shared_capture_thread, s);
  } else {
    threads.emplace_back(capture_thread, &s->road_cam);
    threads.emplace_back(capture_thread, &s->driver_cam);
  }
  threads.emplace_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (dm_enabled) {
    threads.emplace_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  }

  for (auto &t : threads) {
    t.join();
  }

  cameras_close(s);
}
