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
  input_frames_nhwc = std::make_unique<float[]>(buf_size);
  current_frame_nhwc = std::make_unique<float[]>(MODEL_FRAME_SIZE);
  prev_frame_nhwc = std::make_unique<float[]>(MODEL_FRAME_SIZE);
  std::fill_n(input_frames.get(), buf_size, 0.0f);
  std::fill_n(input_frames_nhwc.get(), buf_size, 0.0f);
  std::fill_n(current_frame_nhwc.get(), MODEL_FRAME_SIZE, 0.0f);
  std::fill_n(prev_frame_nhwc.get(), MODEL_FRAME_SIZE, 0.0f);

  q = CL_CHECK_ERR(clCreateCommandQueue(context, device_id, 0, &err));
  y_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_WIDTH * MODEL_HEIGHT, NULL, &err));
  u_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  v_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  net_input_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_FRAME_SIZE * sizeof(float), NULL, &err));
  net_input_nhwc_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_FRAME_SIZE * sizeof(float), NULL, &err));

  transform_init(&transform, context, device_id);
  loadyuv_init(&loadyuv, context, device_id, MODEL_WIDTH, MODEL_HEIGHT);
}

float* ModelFrame::prepare(cl_mem yuv_cl, int frame_width, int frame_height, const mat3 &projection, cl_mem *output, bool nhwc_output) {
  if (output == NULL) {
    if (nhwc_output) {
      loadyuv_queue_nhwc_fused(&loadyuv, q, yuv_cl, frame_width, frame_height, projection, net_input_nhwc_cl);
      CL_CHECK(clEnqueueReadBuffer(q, net_input_nhwc_cl, CL_TRUE, 0, MODEL_FRAME_SIZE * sizeof(float), current_frame_nhwc.get(), 0, nullptr, nullptr));

      const int pixels = (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2);
      for (int i = 0; i < pixels; ++i) {
        float *dst = &input_frames_nhwc[i * 12];
        const float *prev = &prev_frame_nhwc[i * 6];
        const float *cur = &current_frame_nhwc[i * 6];
        std::memcpy(dst, prev, 6 * sizeof(float));
        std::memcpy(dst + 6, cur, 6 * sizeof(float));
      }
      std::swap(prev_frame_nhwc, current_frame_nhwc);
      return input_frames_nhwc.get();
    }

    transform_queue(&this->transform, q,
                    yuv_cl, frame_width, frame_height,
                    y_cl, u_cl, v_cl, MODEL_WIDTH, MODEL_HEIGHT, projection);
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, net_input_cl);

    std::memmove(&input_frames[0], &input_frames[MODEL_FRAME_SIZE], sizeof(float) * MODEL_FRAME_SIZE);
    CL_CHECK(clEnqueueReadBuffer(q, net_input_cl, CL_TRUE, 0, MODEL_FRAME_SIZE * sizeof(float), &input_frames[MODEL_FRAME_SIZE], 0, nullptr, nullptr));
    clFinish(q);
    return &input_frames[0];
  } else {
    transform_queue(&this->transform, q,
                    yuv_cl, frame_width, frame_height,
                    y_cl, u_cl, v_cl, MODEL_WIDTH, MODEL_HEIGHT, projection);
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, *output, true);
    return NULL;
  }
}

void ModelFrame::finish() {
  clFinish(q);
}

ModelFrame::~ModelFrame() {
  transform_destroy(&transform);
  loadyuv_destroy(&loadyuv);
  CL_CHECK(clReleaseMemObject(net_input_nhwc_cl));
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
