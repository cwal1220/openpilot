#pragma once
#ifndef __V4L2_DRM_H__
#define __V4L2_DRM_H__

#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct v4l2_drm_video_buffer {
  void *mmap;
  int fd;
  unsigned index;
};

struct v4l2_crop_size {
  uint32_t width;
  uint32_t height;
  uint32_t offset_x;
  uint32_t offset_y;
};

struct v4l2_drm_context {
  unsigned width;
  unsigned height;
  unsigned device;
  int video_fd;
  uint32_t video_format;
  unsigned buffer_num;
  struct v4l2_drm_video_buffer *buffers;
  struct v4l2_crop_size crop_size;
  struct v4l2_buffer vbuffer;
};

void v4l2_drm_default_context(struct v4l2_drm_context *ctx);
int v4l2_drm_setup(struct v4l2_drm_context context[], unsigned num);
int v4l2_drm_start(const struct v4l2_drm_context *context);
int v4l2_drm_stop(const struct v4l2_drm_context *context);
int v4l2_drm_dump(struct v4l2_drm_context *context, int timeout);
int v4l2_drm_dump_release(struct v4l2_drm_context *context);

#ifdef __cplusplus
}
#endif

#endif
