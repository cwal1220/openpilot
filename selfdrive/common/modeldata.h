#pragma once

#include <array>
#include "selfdrive/common/mat.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

const int  TRAJECTORY_SIZE = 33;
const int LAT_MPC_N = 16;
const int LON_MPC_N = 32;
const float MIN_DRAW_DISTANCE = 10.0;
const float MAX_DRAW_DISTANCE = 100.0;

template <typename T, size_t size>
constexpr std::array<T, size> build_idxs(float max_val) {
  std::array<T, size> result{};
  for (int i = 0; i < size; ++i) {
    result[i] = max_val * ((i / (double)(size - 1)) * (i / (double)(size - 1)));
  }
  return result;
}

constexpr auto T_IDXS = build_idxs<double, TRAJECTORY_SIZE>(10.0);
constexpr auto T_IDXS_FLOAT = build_idxs<float, TRAJECTORY_SIZE>(10.0);
constexpr auto X_IDXS = build_idxs<double, TRAJECTORY_SIZE>(192.0);
constexpr auto X_IDXS_FLOAT = build_idxs<float, TRAJECTORY_SIZE>(192.0);

const int TICI_CAM_WIDTH = 1928;

namespace tici_dm_crop {
  const int x_offset = -72;
  const int y_offset = -144;
  const int width = 954;
};

static inline mat3 get_webcam_fcam_intrinsic_matrix() {
  const float width = util::getenv("ROADCAM_WIDTH", 1280);
  const float height = util::getenv("ROADCAM_HEIGHT", 720);
  const float default_focal = width * (910.0f / 1164.0f);
  const float fx = util::getenv("WEBCAM_FX", util::getenv("WEBCAM_FOCAL", default_focal));
  const float fy = util::getenv("WEBCAM_FY", util::getenv("WEBCAM_FOCAL", default_focal));
  const float cx = util::getenv("WEBCAM_CX", width / 2.0f);
  const float cy = util::getenv("WEBCAM_CY", height / 2.0f);
  return (mat3){{fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0}};
}

const mat3 fcam_intrinsic_matrix =
    Hardware::EON() ? (mat3){{910., 0., 1164.0 / 2,
                              0., 910., 874.0 / 2,
                              0., 0., 1.}}
                    : (Hardware::PC() && util::getenv("USE_WEBCAM", 0) == 1) ? get_webcam_fcam_intrinsic_matrix()
                    : (mat3){{2648.0, 0.0, 1928.0 / 2,
                              0.0, 2648.0, 1208.0 / 2,
                              0.0, 0.0, 1.0}};

// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
const mat3 ecam_intrinsic_matrix = (mat3){{567.0, 0.0, 1928.0 / 2,
                                           0.0, 567.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};

static inline mat3 get_model_yuv_transform(bool bayer = true) {
  float db_s = Hardware::EON() ? 0.5 : 1.0; // debayering does a 2x downscale on EON
  const mat3 transform = (mat3){{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  }};
  return bayer ? transform_scale_buffer(transform, db_s) : transform;
}
