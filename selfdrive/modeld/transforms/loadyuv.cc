#include "selfdrive/modeld/transforms/loadyuv.h"

#include <cassert>
#include <cstdio>
#include <cstring>

#include "selfdrive/common/mat.h"

void loadyuv_init(LoadYUVState* s, cl_context ctx, cl_device_id device_id, int width, int height) {
  memset(s, 0, sizeof(*s));

  s->width = width;
  s->height = height;

  char args[1024];
  snprintf(args, sizeof(args),
           "-cl-fast-relaxed-math -cl-denorms-are-zero "
           "-DTRANSFORMED_WIDTH=%d -DTRANSFORMED_HEIGHT=%d",
           width, height);
  cl_program prg = cl_program_from_file(ctx, device_id, "transforms/loadyuv.cl", args);

  s->loadys_krnl = CL_CHECK_ERR(clCreateKernel(prg, "loadys", &err));
  s->loaduv_krnl = CL_CHECK_ERR(clCreateKernel(prg, "loaduv", &err));
  s->copy_krnl = CL_CHECK_ERR(clCreateKernel(prg, "copy", &err));
  s->loadyuv_nhwc_krnl = CL_CHECK_ERR(clCreateKernel(prg, "loadyuv_nhwc", &err));
  s->warp_loadyuv_nhwc_krnl = CL_CHECK_ERR(clCreateKernel(prg, "warp_loadyuv_nhwc", &err));

  s->m_y_cl = CL_CHECK_ERR(clCreateBuffer(ctx, CL_MEM_READ_WRITE, 3*3*sizeof(float), NULL, &err));
  s->m_uv_cl = CL_CHECK_ERR(clCreateBuffer(ctx, CL_MEM_READ_WRITE, 3*3*sizeof(float), NULL, &err));

  // done with this
  CL_CHECK(clReleaseProgram(prg));
}

void loadyuv_destroy(LoadYUVState* s) {
  CL_CHECK(clReleaseMemObject(s->m_y_cl));
  CL_CHECK(clReleaseMemObject(s->m_uv_cl));
  CL_CHECK(clReleaseKernel(s->loadys_krnl));
  CL_CHECK(clReleaseKernel(s->loaduv_krnl));
  CL_CHECK(clReleaseKernel(s->copy_krnl));
  CL_CHECK(clReleaseKernel(s->loadyuv_nhwc_krnl));
  CL_CHECK(clReleaseKernel(s->warp_loadyuv_nhwc_krnl));
}

void loadyuv_queue(LoadYUVState* s, cl_command_queue q,
                   cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                   cl_mem out_cl, bool do_shift) {
  cl_int global_out_off = 0;
  if (do_shift) {
    // shift the image in slot 1 to slot 0, then place the new image in slot 1
    global_out_off += (s->width*s->height) + (s->width/2)*(s->height/2)*2;
    CL_CHECK(clSetKernelArg(s->copy_krnl, 0, sizeof(cl_mem), &out_cl));
    CL_CHECK(clSetKernelArg(s->copy_krnl, 1, sizeof(cl_int), &global_out_off));
    const size_t copy_work_size = global_out_off/8;
    CL_CHECK(clEnqueueNDRangeKernel(q, s->copy_krnl, 1, NULL,
                                &copy_work_size, NULL, 0, 0, NULL));
  }

  CL_CHECK(clSetKernelArg(s->loadys_krnl, 0, sizeof(cl_mem), &y_cl));
  CL_CHECK(clSetKernelArg(s->loadys_krnl, 1, sizeof(cl_mem), &out_cl));
  CL_CHECK(clSetKernelArg(s->loadys_krnl, 2, sizeof(cl_int), &global_out_off));

  const size_t loadys_work_size = (s->width*s->height)/8;
  CL_CHECK(clEnqueueNDRangeKernel(q, s->loadys_krnl, 1, NULL,
                               &loadys_work_size, NULL, 0, 0, NULL));

  const size_t loaduv_work_size = ((s->width/2)*(s->height/2))/8;
  global_out_off += (s->width*s->height);

  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 0, sizeof(cl_mem), &u_cl));
  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 1, sizeof(cl_mem), &out_cl));
  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 2, sizeof(cl_int), &global_out_off));

  CL_CHECK(clEnqueueNDRangeKernel(q, s->loaduv_krnl, 1, NULL,
                               &loaduv_work_size, NULL, 0, 0, NULL));

  global_out_off += (s->width/2)*(s->height/2);

  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 0, sizeof(cl_mem), &v_cl));
  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 1, sizeof(cl_mem), &out_cl));
  CL_CHECK(clSetKernelArg(s->loaduv_krnl, 2, sizeof(cl_int), &global_out_off));

  CL_CHECK(clEnqueueNDRangeKernel(q, s->loaduv_krnl, 1, NULL,
                               &loaduv_work_size, NULL, 0, 0, NULL));
}

void loadyuv_queue_nhwc(LoadYUVState* s, cl_command_queue q,
                        cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                        cl_mem out_cl) {
  CL_CHECK(clSetKernelArg(s->loadyuv_nhwc_krnl, 0, sizeof(cl_mem), &y_cl));
  CL_CHECK(clSetKernelArg(s->loadyuv_nhwc_krnl, 1, sizeof(cl_mem), &u_cl));
  CL_CHECK(clSetKernelArg(s->loadyuv_nhwc_krnl, 2, sizeof(cl_mem), &v_cl));
  CL_CHECK(clSetKernelArg(s->loadyuv_nhwc_krnl, 3, sizeof(cl_mem), &out_cl));

  const size_t work_size = (s->width / 2) * (s->height / 2);
  CL_CHECK(clEnqueueNDRangeKernel(q, s->loadyuv_nhwc_krnl, 1, NULL,
                               &work_size, NULL, 0, 0, NULL));
}

void loadyuv_queue_nhwc_fused(LoadYUVState* s, cl_command_queue q,
                              cl_mem yuv_cl, int in_width, int in_height,
                              const mat3& projection, cl_mem out_cl) {
  mat3 projection_y = projection;
  mat3 projection_uv = transform_scale_buffer(projection, 0.5);

  CL_CHECK(clEnqueueWriteBuffer(q, s->m_y_cl, CL_TRUE, 0, 3*3*sizeof(float), (void*)projection_y.v, 0, NULL, NULL));
  CL_CHECK(clEnqueueWriteBuffer(q, s->m_uv_cl, CL_TRUE, 0, 3*3*sizeof(float), (void*)projection_uv.v, 0, NULL, NULL));

  const int in_y_width = in_width;
  const int in_y_height = in_height;
  const int in_uv_width = in_width / 2;
  const int in_uv_height = in_height / 2;
  const int in_y_offset = 0;
  const int in_u_offset = in_y_offset + in_y_width * in_y_height;
  const int in_v_offset = in_u_offset + in_uv_width * in_uv_height;

  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 0, sizeof(cl_mem), &yuv_cl));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 1, sizeof(cl_int), &in_y_width));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 2, sizeof(cl_int), &in_y_height));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 3, sizeof(cl_int), &in_uv_width));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 4, sizeof(cl_int), &in_uv_height));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 5, sizeof(cl_int), &in_y_offset));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 6, sizeof(cl_int), &in_u_offset));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 7, sizeof(cl_int), &in_v_offset));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 8, sizeof(cl_mem), &out_cl));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 9, sizeof(cl_mem), &s->m_y_cl));
  CL_CHECK(clSetKernelArg(s->warp_loadyuv_nhwc_krnl, 10, sizeof(cl_mem), &s->m_uv_cl));

  const size_t work_size[2] = {(size_t)(s->width / 2), (size_t)(s->height / 2)};
  CL_CHECK(clEnqueueNDRangeKernel(q, s->warp_loadyuv_nhwc_krnl, 2, NULL,
                               (const size_t*)&work_size, NULL, 0, 0, NULL));
}
