#include "selfdrive/modeld/models/commonmodel.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

#include "selfdrive/common/clutil.h"
#include "selfdrive/common/mat.h"
#include "selfdrive/common/timing.h"

ModelFrame::ModelFrame(cl_device_id device_id, cl_context context) {
  input_frames = std::make_unique<float[]>(buf_size);

  if (device_id == nullptr || context == nullptr) {
    use_cl = false;
    return;
  }

  use_cl = true;
  q = CL_CHECK_ERR(clCreateCommandQueue(context, device_id, 0, &err));
  y_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_WIDTH * MODEL_HEIGHT, NULL, &err));
  u_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  v_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  net_input_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_FRAME_SIZE * sizeof(float), NULL, &err));

  transform_init(&transform, context, device_id);
  loadyuv_init(&loadyuv, context, device_id, MODEL_WIDTH, MODEL_HEIGHT);
}

uint8_t ModelFrame::sample_plane_mapped(const uint8_t *src, const CpuWarpSample &sample) const {
  constexpr int INTER_REMAP_COEF_BITS = 15;

  int val = 0;
  val += static_cast<int>(src[sample.offset[0]]) * sample.weight[0];
  val += static_cast<int>(src[sample.offset[1]]) * sample.weight[1];
  val += static_cast<int>(src[sample.offset[2]]) * sample.weight[2];
  val += static_cast<int>(src[sample.offset[3]]) * sample.weight[3];
  return static_cast<uint8_t>(std::clamp((val + (1 << (INTER_REMAP_COEF_BITS - 1))) >> INTER_REMAP_COEF_BITS, 0, 255));
}

void ModelFrame::build_cpu_warp_map(const mat3 &projection, int src_w, int src_h, int dst_w, int dst_h,
                                    std::vector<CpuWarpSample> &map) {
  constexpr int INTER_BITS = 5;
  constexpr int INTER_TAB_SIZE = 1 << INTER_BITS;
  constexpr int INTER_REMAP_COEF_BITS = 15;
  constexpr int INTER_REMAP_COEF_SCALE = 1 << INTER_REMAP_COEF_BITS;

  map.resize(static_cast<size_t>(dst_w) * dst_h);
  for (int dy = 0; dy < dst_h; ++dy) {
    for (int dx = 0; dx < dst_w; ++dx) {
      CpuWarpSample sample = {};
      float X0 = projection.v[0] * dx + projection.v[1] * dy + projection.v[2];
      float Y0 = projection.v[3] * dx + projection.v[4] * dy + projection.v[5];
      const float W0 = projection.v[6] * dx + projection.v[7] * dy + projection.v[8];
      if (std::fabs(W0) <= 1e-6f) {
        map[static_cast<size_t>(dy) * dst_w + dx] = sample;
        continue;
      }

      const float src_x = X0 / W0;
      const float src_y = Y0 / W0;
      if (!std::isfinite(src_x) || !std::isfinite(src_y) ||
          src_x < -1.0f || src_x >= static_cast<float>(src_w) ||
          src_y < -1.0f || src_y >= static_cast<float>(src_h)) {
        map[static_cast<size_t>(dy) * dst_w + dx] = sample;
        continue;
      }

      const int X = static_cast<int>(std::nearbyint(src_x * INTER_TAB_SIZE));
      const int Y = static_cast<int>(std::nearbyint(src_y * INTER_TAB_SIZE));
      const int sx = X >> INTER_BITS;
      const int sy = Y >> INTER_BITS;
      const int ax = X & (INTER_TAB_SIZE - 1);
      const int ay = Y & (INTER_TAB_SIZE - 1);

      const float tabx = static_cast<float>(ax) / INTER_TAB_SIZE;
      const float taby = static_cast<float>(ay) / INTER_TAB_SIZE;
      const int weights[4] = {
        static_cast<int>(std::nearbyint((1.0f - taby) * (1.0f - tabx) * INTER_REMAP_COEF_SCALE)),
        static_cast<int>(std::nearbyint((1.0f - taby) * tabx * INTER_REMAP_COEF_SCALE)),
        static_cast<int>(std::nearbyint(taby * (1.0f - tabx) * INTER_REMAP_COEF_SCALE)),
        static_cast<int>(std::nearbyint(taby * tabx * INTER_REMAP_COEF_SCALE)),
      };
      const int xs[4] = {sx, sx + 1, sx, sx + 1};
      const int ys[4] = {sy, sy, sy + 1, sy + 1};

      for (int i = 0; i < 4; ++i) {
        if (xs[i] >= 0 && xs[i] < src_w && ys[i] >= 0 && ys[i] < src_h) {
          sample.offset[i] = static_cast<uint32_t>(ys[i] * src_w + xs[i]);
          sample.weight[i] = static_cast<uint16_t>(std::clamp(weights[i], 0, INTER_REMAP_COEF_SCALE));
        }
      }
      map[static_cast<size_t>(dy) * dst_w + dx] = sample;
    }
  }
}

bool ModelFrame::cpu_warp_maps_valid(VisionBuf *buf, const mat3 &projection) const {
  return cpu_map_valid &&
         cpu_map_width == static_cast<int>(buf->width) &&
         cpu_map_height == static_cast<int>(buf->height) &&
         std::memcmp(cpu_map_projection.v, projection.v, sizeof(projection.v)) == 0;
}

void ModelFrame::rebuild_cpu_warp_maps(VisionBuf *buf, const mat3 &projection) {
  const mat3 projection_uv = transform_scale_buffer(projection, 0.5);
  build_cpu_warp_map(projection, static_cast<int>(buf->width), static_cast<int>(buf->height),
                     MODEL_WIDTH, MODEL_HEIGHT, cpu_y_map);
  build_cpu_warp_map(projection_uv, static_cast<int>(buf->width / 2), static_cast<int>(buf->height / 2),
                     MODEL_WIDTH / 2, MODEL_HEIGHT / 2, cpu_uv_map);
  cpu_map_width = static_cast<int>(buf->width);
  cpu_map_height = static_cast<int>(buf->height);
  cpu_map_projection = projection;
  cpu_map_valid = true;
}

void ModelFrame::prepare_cpu(VisionBuf *buf, const mat3 &projection, float *out) {
  if (!cpu_warp_maps_valid(buf, projection)) {
    rebuild_cpu_warp_maps(buf, projection);
  }

  const int half_w = MODEL_WIDTH / 2;
  const int half_h = MODEL_HEIGHT / 2;
  const int plane_size = half_w * half_h;

  float *y00 = out;
  float *y10 = y00 + plane_size;
  float *y01 = y10 + plane_size;
  float *y11 = y01 + plane_size;
  float *u = y11 + plane_size;
  float *v = u + plane_size;

  for (int y2 = 0; y2 < half_h; ++y2) {
    for (int x2 = 0; x2 < half_w; ++x2) {
      const int ox = x2 * 2;
      const int oy = y2 * 2;
      const int dst = y2 * half_w + x2;

      y00[dst] = static_cast<float>(sample_plane_mapped(buf->y, cpu_y_map[static_cast<size_t>(oy) * MODEL_WIDTH + ox]));
      y10[dst] = static_cast<float>(sample_plane_mapped(buf->y, cpu_y_map[static_cast<size_t>(oy + 1) * MODEL_WIDTH + ox]));
      y01[dst] = static_cast<float>(sample_plane_mapped(buf->y, cpu_y_map[static_cast<size_t>(oy) * MODEL_WIDTH + ox + 1]));
      y11[dst] = static_cast<float>(sample_plane_mapped(buf->y, cpu_y_map[static_cast<size_t>(oy + 1) * MODEL_WIDTH + ox + 1]));
      u[dst] = static_cast<float>(sample_plane_mapped(buf->u, cpu_uv_map[dst]));
      v[dst] = static_cast<float>(sample_plane_mapped(buf->v, cpu_uv_map[dst]));
    }
  }
}

float* ModelFrame::prepare(VisionBuf *buf, const mat3 &projection, cl_mem *output) {
  if (!use_cl) {
    assert(output == nullptr);
    std::memmove(&input_frames[0], &input_frames[MODEL_FRAME_SIZE], sizeof(float) * MODEL_FRAME_SIZE);
    prepare_cpu(buf, projection, &input_frames[MODEL_FRAME_SIZE]);
    return &input_frames[0];
  }

  cl_mem yuv_cl = buf->buf_cl;
  int frame_width = buf->width;
  int frame_height = buf->height;
  transform_queue(&this->transform, q,
                  yuv_cl, frame_width, frame_height,
                  y_cl, u_cl, v_cl, MODEL_WIDTH, MODEL_HEIGHT, projection);

  if (output == NULL) {
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, net_input_cl);

    std::memmove(&input_frames[0], &input_frames[MODEL_FRAME_SIZE], sizeof(float) * MODEL_FRAME_SIZE);
    CL_CHECK(clEnqueueReadBuffer(q, net_input_cl, CL_TRUE, 0, MODEL_FRAME_SIZE * sizeof(float), &input_frames[MODEL_FRAME_SIZE], 0, nullptr, nullptr));
    clFinish(q);
    return &input_frames[0];
  } else {
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, *output, true);
    return NULL;
  }
}

void ModelFrame::finish() {
  if (use_cl) clFinish(q);
}

ModelFrame::~ModelFrame() {
  if (!use_cl) return;
  transform_destroy(&transform);
  loadyuv_destroy(&loadyuv);
  CL_CHECK(clReleaseMemObject(net_input_cl));
  CL_CHECK(clReleaseMemObject(v_cl));
  CL_CHECK(clReleaseMemObject(u_cl));
  CL_CHECK(clReleaseMemObject(y_cl));
  CL_CHECK(clReleaseCommandQueue(q));
}

void softmax(const float* input, float* output, size_t len) {
  const float max_val = *std::max_element(input, input + len);
  float denominator = 0;
  for(int i = 0; i < len; i++) {
    float const v_exp = expf(input[i] - max_val);
    denominator += v_exp;
    output[i] = v_exp;
  }

  const float inv_denominator = 1. / denominator;
  for(int i = 0; i < len; i++) {
    output[i] *= inv_denominator;
  }
}

float sigmoid(float input) {
  return 1 / (1 + expf(-input));
}

float softplus(float input) {
  return log1p(expf(input));
}
