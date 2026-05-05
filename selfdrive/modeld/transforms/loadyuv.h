#pragma once

#include "selfdrive/common/clutil.h"
#include "selfdrive/common/mat.h"

typedef struct {
  int width, height;
  cl_kernel loadys_krnl, loaduv_krnl, copy_krnl, loadyuv_nhwc_krnl, warp_loadyuv_nhwc_krnl;
  cl_mem m_y_cl, m_uv_cl;
} LoadYUVState;

void loadyuv_init(LoadYUVState* s, cl_context ctx, cl_device_id device_id, int width, int height);

void loadyuv_destroy(LoadYUVState* s);

void loadyuv_queue(LoadYUVState* s, cl_command_queue q,
                   cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                   cl_mem out_cl, bool do_shift = false);

void loadyuv_queue_nhwc(LoadYUVState* s, cl_command_queue q,
                        cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                        cl_mem out_cl);

void loadyuv_queue_nhwc_fused(LoadYUVState* s, cl_command_queue q,
                              cl_mem yuv_cl, int in_width, int in_height,
                              const mat3& projection, cl_mem out_cl);
