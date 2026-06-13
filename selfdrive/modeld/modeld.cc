#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <cmath>
#include <algorithm>

#include <eigen3/Eigen/Dense>

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/clutil.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/modeld/models/driving.h"

ExitHandler do_exit;

#ifdef USE_K230_KMODEL
namespace {

constexpr int K230_OV5647_IMAGE_WIDTH = 1920;
constexpr int K230_OV5647_IMAGE_HEIGHT = 1080;
// OV5647 calibration for the K230 board camera at 1920x1080.
// Distortion: [0.1248010166, -0.9305102337, 0.0007435436, 0.0005325169, 1.3074202906].
// The current K230 path does not undistort, so modeld uses the pinhole camera matrix only.
constexpr float K230_OV5647_FX = 1625.7416788144435f;
constexpr float K230_OV5647_FY = 1585.9830269782024f;
constexpr float K230_OV5647_CX = 946.13450988811394f;
constexpr float K230_OV5647_CY = 537.34063862123787f;
constexpr float K230_MIN_WARP_INBOUNDS = 0.85f;
constexpr float K230_RECURRENT_RESET_TRANSFORM_DELTA = 5.0f;
constexpr float K230_MAX_CALIB_RPY_RAD = 2.5f * 0.01745329252f;

Eigen::Matrix<float, 3, 4> k230_zero_extrinsics() {
  Eigen::Matrix<float, 3, 4> extrinsics;
  extrinsics << 0.0f, -1.0f,  0.0f, 0.0f,
                0.0f,  0.0f, -1.0f, 1.22f,
                1.0f,  0.0f,  0.0f, 0.0f;
  return extrinsics;
}

mat3 k230_source_intrinsics(int source_width, int source_height) {
  const float sx = static_cast<float>(source_width) / K230_OV5647_IMAGE_WIDTH;
  const float sy = static_cast<float>(source_height) / K230_OV5647_IMAGE_HEIGHT;
  const float fx = K230_OV5647_FX * sx;
  const float fy = K230_OV5647_FY * sy;
  const float cx = K230_OV5647_CX * sx;
  const float cy = K230_OV5647_CY * sy;

  return (mat3){{
    fx, 0.0f, cx,
    0.0f, fy, cy,
    0.0f, 0.0f, 1.0f,
  }};
}

float k230_transform_inbounds_ratio(const mat3 &projection, int source_width, int source_height) {
  constexpr int model_width = 512;
  constexpr int model_height = 256;
  constexpr int step = 8;

  int valid = 0;
  int total = 0;
  for (int y = 0; y < model_height; y += step) {
    for (int x = 0; x < model_width; x += step) {
      const float x0 = projection.v[0] * x + projection.v[1] * y + projection.v[2];
      const float y0 = projection.v[3] * x + projection.v[4] * y + projection.v[5];
      const float w0 = projection.v[6] * x + projection.v[7] * y + projection.v[8];
      if (std::fabs(w0) > 1e-6f) {
        const float sx = x0 / w0;
        const float sy = y0 / w0;
        if (std::isfinite(sx) && std::isfinite(sy) &&
            sx >= -1.0f && sx < source_width &&
            sy >= -1.0f && sy < source_height) {
          ++valid;
        }
      }
      ++total;
    }
  }
  return total > 0 ? static_cast<float>(valid) / total : 0.0f;
}

float k230_transform_delta(const mat3 &a, const mat3 &b) {
  float delta = 0.0f;
  for (int i = 0; i < 9; ++i) {
    delta = std::max(delta, std::fabs(a.v[i] - b.v[i]));
  }
  return delta;
}

bool k230_calibration_rpy_valid(const capnp::List<float>::Reader &rpy_calib, float max_abs_rad) {
  if (rpy_calib.size() < 3) {
    return true;
  }

  for (uint i = 0; i < 3; ++i) {
    const float value = rpy_calib[i];
    if (!std::isfinite(value) || std::fabs(value) > max_abs_rad) {
      return false;
    }
  }
  return true;
}

}  // namespace
#endif

mat3 update_calibration(const Eigen::Matrix<float, 3, 4> &extrinsics, const mat3 &cam_intrinsics_mat, bool bigmodel_frame) {
  /*
     import numpy as np
     from common.transformations.model import medmodel_frame_from_road_frame
     medmodel_frame_from_ground = medmodel_frame_from_road_frame[:, (0, 1, 3)]
     ground_from_medmodel_frame = np.linalg.inv(medmodel_frame_from_ground)
  */
  static const auto ground_from_medmodel_frame = (Eigen::Matrix<float, 3, 3>() <<
     0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
    -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
    -1.84808520e-20, 9.00738606e-04, -4.28751576e-02).finished();

  static const auto ground_from_sbigmodel_frame = (Eigen::Matrix<float, 3, 3>() <<
     0.00000000e+00,  7.31372216e-19,  1.00000000e+00,
    -2.19780220e-03,  4.11497335e-19,  5.62637363e-01,
    -5.46146580e-20,  1.80147721e-03, -2.73464241e-01).finished();

  static const mat3 yuv_transform = get_model_yuv_transform();
  const auto cam_intrinsics = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(cam_intrinsics_mat.v);

  auto ground_from_model_frame = bigmodel_frame ? ground_from_sbigmodel_frame : ground_from_medmodel_frame;
  auto camera_frame_from_road_frame = cam_intrinsics * extrinsics;
  Eigen::Matrix<float, 3, 3> camera_frame_from_ground;
  camera_frame_from_ground.col(0) = camera_frame_from_road_frame.col(0);
  camera_frame_from_ground.col(1) = camera_frame_from_road_frame.col(1);
  camera_frame_from_ground.col(2) = camera_frame_from_road_frame.col(3);

  auto warp_matrix = camera_frame_from_ground * ground_from_model_frame;
  mat3 transform = {};
  for (int i=0; i<3*3; i++) {
    transform.v[i] = warp_matrix(i / 3, i % 3);
  }
  return matmul3(yuv_transform, transform);
}

static uint64_t get_ts(const VisionIpcBufExtra &extra) {
  return Hardware::TICI() ? extra.timestamp_sof : extra.timestamp_eof;
}

void run_model(ModelState &model, VisionIpcClient &vipc_client_main, VisionIpcClient &vipc_client_extra,
               bool main_wide_camera, bool use_extra_client, int main_width, int main_height) {
  // messaging
  PubMaster pm({"modelV2", "cameraOdometry"});
  SubMaster sm({"lateralPlan", "roadCameraState", "liveCalibration"});

  // setup filter to track dropped frames
  FirstOrderFilter frame_dropped_filter(0., 10., 1. / MODEL_FREQ);

  uint32_t frame_id = 0, last_vipc_frame_id = 0;
  uint32_t run_count = 0;

  mat3 model_transform_main = {};
  mat3 model_transform_extra = {};
  bool live_calib_seen = false;

#ifdef USE_K230_KMODEL
  const mat3 main_cam_intrinsics = k230_source_intrinsics(main_width, main_height);
  const mat3 extra_cam_intrinsics = main_cam_intrinsics;
  const uint64_t model_period_ns = 1000000000ULL / MODEL_FREQ;
  uint64_t next_model_start_ns = 0;
  uint32_t last_calib_reject_log_count = 0;
  bool calib_reject_logged = false;
#else
  (void)main_width;
  (void)main_height;
  const mat3 main_cam_intrinsics = main_wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  const mat3 extra_cam_intrinsics = Hardware::TICI() ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
#endif
  const Eigen::Matrix<float, 3, 4> zero_extrinsics =
#ifdef USE_K230_KMODEL
      k230_zero_extrinsics();
#else
      Eigen::Matrix<float, 3, 4>::Zero();
#endif
  model_transform_main = update_calibration(zero_extrinsics, main_cam_intrinsics, false);
  model_transform_extra = update_calibration(zero_extrinsics, extra_cam_intrinsics, true);

  VisionBuf *buf_main = nullptr;
  VisionBuf *buf_extra = nullptr;

  VisionIpcBufExtra meta_main = {0};
  VisionIpcBufExtra meta_extra = {0};

  while (!do_exit) {
#ifdef USE_K230_KMODEL
    if (next_model_start_ns != 0) {
      const uint64_t now = nanos_since_boot();
      if (now < next_model_start_ns) {
        util::sleep_for(static_cast<int>((next_model_start_ns - now) / 1000000ULL));
      }
    }
#endif

    // Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while (get_ts(meta_main) < get_ts(meta_extra) + 25000000ULL) {
      buf_main = vipc_client_main.recv(&meta_main);
      if (buf_main == nullptr)  break;
    }

    if (buf_main == nullptr) {
      LOGE("vipc_client_main no frame");
      continue;
    }

    if (use_extra_client) {
      // Keep receiving extra frames until frame id matches main camera
      do {
        buf_extra = vipc_client_extra.recv(&meta_extra);
      } while (buf_extra != nullptr && get_ts(meta_main) > get_ts(meta_extra) + 25000000ULL);

      if (buf_extra == nullptr) {
        LOGE("vipc_client_extra no frame");
        continue;
      }

      if (std::abs((int64_t)meta_main.timestamp_sof - (int64_t)meta_extra.timestamp_sof) > 10000000ULL) {
        LOGE("frames out of sync! main: %d (%.5f), extra: %d (%.5f)",
          meta_main.frame_id, double(meta_main.timestamp_sof) / 1e9,
          meta_extra.frame_id, double(meta_extra.timestamp_sof) / 1e9);
      }
    } else {
      // Use single camera
      buf_extra = buf_main;
      meta_extra = meta_main;
    }

#ifdef USE_K230_KMODEL
    if (!use_extra_client) {
      VisionIpcBufExtra latest_meta = meta_main;
      VisionBuf *latest_buf = nullptr;
      while ((latest_buf = vipc_client_main.recv(&latest_meta, 0)) != nullptr) {
        buf_main = latest_buf;
        meta_main = latest_meta;
      }
      buf_extra = buf_main;
      meta_extra = meta_main;
    }
#endif

    // TODO: path planner timeout?
    sm.update(0);
    int desire = ((int)sm["lateralPlan"].getLateralPlan().getDesire());
    frame_id = sm["roadCameraState"].getRoadCameraState().getFrameId();
    if (sm.updated("liveCalibration")) {
      auto live_calibration = sm["liveCalibration"].getLiveCalibration();
      auto extrinsic_matrix = live_calibration.getExtrinsicMatrix();
      Eigen::Matrix<float, 3, 4> extrinsic_matrix_eigen;
      for (int i = 0; i < 4*3; i++) {
        extrinsic_matrix_eigen(i / 4, i % 4) = extrinsic_matrix[i];
      }

      const mat3 candidate_transform_main = update_calibration(extrinsic_matrix_eigen, main_cam_intrinsics, false);
      const mat3 candidate_transform_extra = update_calibration(extrinsic_matrix_eigen, extra_cam_intrinsics, true);
#ifdef USE_K230_KMODEL
      const bool rpy_valid = k230_calibration_rpy_valid(live_calibration.getRpyCalib(), K230_MAX_CALIB_RPY_RAD);
      const float inbounds = k230_transform_inbounds_ratio(candidate_transform_main, main_width, main_height);
      if (rpy_valid && inbounds >= K230_MIN_WARP_INBOUNDS) {
        const float transform_delta = k230_transform_delta(model_transform_main, candidate_transform_main);
        if (transform_delta > K230_RECURRENT_RESET_TRANSFORM_DELTA) {
          model_reset_recurrent(&model);
          LOGW("resetting K230 recurrent state after calibration transform delta %.3f", transform_delta);
        }
        model_transform_main = candidate_transform_main;
        model_transform_extra = candidate_transform_extra;
        live_calib_seen = true;
      } else {
        model_reset_recurrent(&model);
        if (!calib_reject_logged || run_count - last_calib_reject_log_count >= MODEL_FREQ * 5) {
          LOGW("rejecting K230 calibration rpy_valid=%d warp inbounds %.1f%% min %.1f%%",
               rpy_valid, inbounds * 100.0f, K230_MIN_WARP_INBOUNDS * 100.0f);
          last_calib_reject_log_count = run_count;
          calib_reject_logged = true;
        }
      }
#else
      model_transform_main = candidate_transform_main;
      model_transform_extra = candidate_transform_extra;
      live_calib_seen = true;
#endif
    }

    float vec_desire[DESIRE_LEN] = {0};
    if (desire >= 0 && desire < DESIRE_LEN) {
      vec_desire[desire] = 1.0;
    }

    // tracked dropped frames
    uint32_t vipc_dropped_frames = meta_main.frame_id - last_vipc_frame_id - 1;
    float frames_dropped = frame_dropped_filter.update((float)std::min(vipc_dropped_frames, 10U));
    if (run_count < 10) { // let frame drops warm up
      frame_dropped_filter.reset(0);
      frames_dropped = 0.;
    }
    run_count++;

    float frame_drop_ratio = frames_dropped / (1 + frames_dropped);

#ifdef USE_K230_KMODEL
    const uint64_t model_start_ns = nanos_since_boot();
    double mt1 = static_cast<double>(model_start_ns) / 1000000.0;
#else
    double mt1 = millis_since_boot();
#endif
    ModelOutput *model_output = model_eval_frame(&model, buf_main, buf_extra, model_transform_main, model_transform_extra, vec_desire);
    double mt2 = millis_since_boot();
    float model_execution_time = (mt2 - mt1) / 1000.0;

    model_publish(pm, meta_main.frame_id, meta_extra.frame_id, frame_id, frame_drop_ratio, *model_output, meta_main.timestamp_eof, model_execution_time,
                  kj::ArrayPtr<const float>(model.output.data(), model.output.size()), live_calib_seen);
    posenet_publish(pm, meta_main.frame_id, vipc_dropped_frames, *model_output, meta_main.timestamp_eof, live_calib_seen);

    last_vipc_frame_id = meta_main.frame_id;

#ifdef USE_K230_KMODEL
    if (next_model_start_ns == 0 || model_start_ns >= next_model_start_ns + model_period_ns) {
      next_model_start_ns = model_start_ns + model_period_ns;
    } else {
      next_model_start_ns += model_period_ns;
    }
#endif
  }
}

int main(int argc, char **argv) {
#ifndef USE_K230_KMODEL
  if (!Hardware::PC()) {
    int ret;
    ret = util::set_realtime_priority(54);
    assert(ret == 0);
    util::set_core_affinity({Hardware::EON() ? 2 : 7});
    assert(ret == 0);
  }
#endif

  bool main_wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;
  bool use_extra_client = Hardware::TICI() && !main_wide_camera;

  // cl init
  cl_device_id device_id = nullptr;
  cl_context context = nullptr;
#ifndef USE_K230_KMODEL
  device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
#endif

  // init the models
  ModelState model;
  model_init(&model, device_id, context);
  LOGW("models loaded, modeld starting");

  VisionIpcClient vipc_client_main = VisionIpcClient("camerad", main_wide_camera ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD, true, device_id, context);
  VisionIpcClient vipc_client_extra = VisionIpcClient("camerad", VISION_STREAM_WIDE_ROAD, false, device_id, context);

  while (!do_exit && !vipc_client_main.connect(false)) {
    util::sleep_for(100);
  }

  while (!do_exit && use_extra_client && !vipc_client_extra.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  // vipc_client.connected is false only when do_exit is true
  if (!do_exit) {
    const VisionBuf *b = &vipc_client_main.buffers[0];
    LOGW("connected main cam with buffer size: %d (%d x %d)", b->len, b->width, b->height);

    if (use_extra_client) {
      const VisionBuf *wb = &vipc_client_extra.buffers[0];
      LOGW("connected extra cam with buffer size: %d (%d x %d)", wb->len, wb->width, wb->height);
    }

    run_model(model, vipc_client_main, vipc_client_extra, main_wide_camera, use_extra_client,
              b->width, b->height);
  }

  model_free(&model);
  if (context != nullptr) {
    CL_CHECK(clReleaseContext(context));
  }
  return 0;
}
