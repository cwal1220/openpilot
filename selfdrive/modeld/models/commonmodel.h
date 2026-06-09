#pragma once

#include <cfloat>
#include <cstdint>
#include <cstdlib>

#include <memory>
#include <vector>

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "selfdrive/common/mat.h"
#include "cereal/visionipc/visionbuf.h"
#include "selfdrive/modeld/transforms/loadyuv.h"
#include "selfdrive/modeld/transforms/transform.h"

const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;

void softmax(const float* input, float* output, size_t len);
float softplus(float input);
float sigmoid(float input);

class ModelFrame {
public:
  ModelFrame(cl_device_id device_id, cl_context context);
  ~ModelFrame();
  float* prepare(VisionBuf *buf, const mat3& transform, cl_mem *output);
  void finish();

  const int MODEL_WIDTH = 512;
  const int MODEL_HEIGHT = 256;
  const int MODEL_FRAME_SIZE = MODEL_WIDTH * MODEL_HEIGHT * 3 / 2;
  const int buf_size = MODEL_FRAME_SIZE * 2;

private:
  struct CpuWarpSample {
    uint32_t offset[4] = {};
    uint16_t weight[4] = {};
  };

  uint8_t sample_plane_mapped(const uint8_t *src, const CpuWarpSample &sample) const;
  void build_cpu_warp_map(const mat3 &projection, int src_w, int src_h, int dst_w, int dst_h,
                          std::vector<CpuWarpSample> &map);
  void rebuild_cpu_warp_maps(VisionBuf *buf, const mat3 &projection);
  bool cpu_warp_maps_valid(VisionBuf *buf, const mat3 &projection) const;
  void prepare_cpu(VisionBuf *buf, const mat3 &projection, float *out);

  bool use_cl = false;
  Transform transform;
  LoadYUVState loadyuv;
  cl_command_queue q;
  cl_mem y_cl, u_cl, v_cl, net_input_cl;
  std::unique_ptr<float[]> input_frames;

  bool cpu_map_valid = false;
  int cpu_map_width = 0;
  int cpu_map_height = 0;
  mat3 cpu_map_projection = {};
  std::vector<CpuWarpSample> cpu_y_map;
  std::vector<CpuWarpSample> cpu_uv_map;
};
