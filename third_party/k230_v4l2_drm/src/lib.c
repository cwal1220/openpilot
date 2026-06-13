#include "common.h"
#include "v4l2-drm.h"
#include <linux/videodev2.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <unistd.h>

void v4l2_drm_default_context(struct v4l2_drm_context* ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->width = 640;
    ctx->height = 480;
    ctx->device = 0;
    ctx->video_fd = -1;
    ctx->video_format = V4L2_PIX_FMT_NV12;
    ctx->buffer_num = 5;
}

static void v4l2_drm_release_mmap_buffers(struct v4l2_drm_context *ctx);

int v4l2_drm_setup(struct v4l2_drm_context context[], unsigned num) {
    for (unsigned i = 0; i < num; i++) {
        context[i].buffers = NULL;
        char cam_device_path[64];
        snprintf(cam_device_path, sizeof(cam_device_path) - 1, "/dev/video%u", context[i].device);
        context[i].video_fd = open(cam_device_path, O_RDWR | O_NONBLOCK);
        CKE(context[i].video_fd < 0, close);

        struct v4l2_capability capbility;
        CKE(ioctl(context[i].video_fd, VIDIOC_QUERYCAP, &capbility), close);

        struct v4l2_format format;
        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(context[i].video_fd, VIDIOC_G_FMT, &format);
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = context[i].video_format;
        format.fmt.pix.width = context[i].width;
        format.fmt.pix.height = context[i].height;
        CKE(ioctl(context[i].video_fd, VIDIOC_S_FMT, &format), close);

        if((context[i].crop_size.height != 0) && (context[i].crop_size.width != 0) &&
                (context[i].crop_size.height > context[i].height ) && (context[i].crop_size.width > context[i].width))
        {
            struct v4l2_selection sel = {
                .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
                .target = V4L2_SEL_TGT_COMPOSE_BOUNDS,
            };
            struct v4l2_rect r;
            int ret = 0;

            ret = ioctl(context[i].video_fd, VIDIOC_G_SELECTION, &sel);
            if(ret < 0)
                pr("VIDIOC_G_SELECTION error %d(%s)", errno, strerror(errno));

            r.width = context[i].crop_size.width;
            r.height = context[i].crop_size.height;
            r.left = context[i].crop_size.offset_x;
            r.top = context[i].crop_size.offset_y;

            sel.r = r;
            sel.target = V4L2_SEL_TGT_COMPOSE;
            sel.flags = V4L2_SEL_FLAG_LE;

            ret = ioctl(context[i].video_fd, VIDIOC_S_SELECTION, &sel);
            if(ret < 0)
                pr("VIDIOC_S_SELECTION error %d(%s)", errno, strerror(errno));
        }

        struct v4l2_requestbuffers request_buffer;
        memset(&request_buffer, 0, sizeof(request_buffer));
        request_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request_buffer.memory = V4L2_MEMORY_MMAP;
        request_buffer.count = context[i].buffer_num;
        CKE(ioctl(context[i].video_fd, VIDIOC_REQBUFS, &request_buffer), close);
        context[i].buffers = malloc(sizeof(struct v4l2_drm_video_buffer) * context[i].buffer_num);
        CKE(context[i].buffers == NULL, close);
        memset(context[i].buffers, 0,
               sizeof(struct v4l2_drm_video_buffer) * context[i].buffer_num);
        for (unsigned bi = 0; bi < context[i].buffer_num; bi++) {
            context[i].buffers[bi].fd = -1;
        }
        for (unsigned j = 0; j < context[i].buffer_num; j++) {
            memset(&context[i].vbuffer, 0, sizeof(context[i].vbuffer));
            context[i].vbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            context[i].vbuffer.memory = V4L2_MEMORY_MMAP;
            context[i].vbuffer.index = j;
            CKE(ioctl(context[i].video_fd, VIDIOC_QUERYBUF, &context[i].vbuffer), close);
            CKE(ioctl(context[i].video_fd, VIDIOC_QBUF, &context[i].vbuffer), close);
            context[i].buffers[j].mmap = mmap(
                NULL,
                context[i].vbuffer.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                context[i].video_fd,
                context[i].vbuffer.m.offset
            );
            CKE(context[i].buffers[j].mmap == MAP_FAILED, close);
            struct v4l2_exportbuffer expbuf;
            memset(&expbuf, 0, sizeof(expbuf));
            expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            expbuf.index = j;
            CKE(ioctl(context[i].video_fd, VIDIOC_EXPBUF, &expbuf), close);
            context[i].buffers[j].fd = expbuf.fd;
            context[i].buffers[j].index = j;
        }
        continue;

        close:
        for (unsigned j = 0; j <= i; j++) {
            v4l2_drm_release_mmap_buffers(&context[j]);
            if (context[j].video_fd >= 0) {
                close(context[j].video_fd);
                context[j].video_fd = -1;
            }
        }
        return -1;
    }
    return 0;
}

int v4l2_drm_start(const struct v4l2_drm_context* context) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(context->video_fd, VIDIOC_STREAMON, &type);
}

static void v4l2_drm_release_mmap_buffers(struct v4l2_drm_context *ctx)
{
    unsigned j;

    if (!ctx->buffers) {
        return;
    }

    for (j = 0; j < ctx->buffer_num; j++) {
        struct v4l2_buffer vb;
        size_t len = 0;

        memset(&vb, 0, sizeof(vb));
        vb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vb.memory = V4L2_MEMORY_MMAP;
        vb.index = j;
        if (ctx->video_fd >= 0 && ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &vb) == 0) {
            len = vb.length;
        } else if (ctx->vbuffer.length) {
            len = ctx->vbuffer.length;
        }

        if (ctx->buffers[j].mmap && ctx->buffers[j].mmap != MAP_FAILED && len) {
            munmap(ctx->buffers[j].mmap, len);
        }
        ctx->buffers[j].mmap = NULL;

        if (ctx->buffers[j].fd >= 0) {
            close(ctx->buffers[j].fd);
            ctx->buffers[j].fd = -1;
        }
    }
    free(ctx->buffers);
    ctx->buffers = NULL;
}

int v4l2_drm_stop(const struct v4l2_drm_context *context)
{
    struct v4l2_drm_context *ctx = (struct v4l2_drm_context *)context;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret = 0;

    if (!ctx || ctx->video_fd < 0) {
        return -1;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);
    v4l2_drm_release_mmap_buffers(ctx);
    close(ctx->video_fd);
    ctx->video_fd = -1;

    return ret;
}

int v4l2_drm_dump(struct v4l2_drm_context* context, int timeout) {
    struct pollfd pf = {
        .events = POLLIN | POLLPRI,
        .fd = context->video_fd,
        .revents = 0
    };
    int ret;
    retry:
    ret = poll(&pf, 1, timeout);
    if ((ret < 0) && (errno == EINTR)) {
        // try again
        goto retry;
    }
    if (ret == 0) {
        errno = EAGAIN;
        return -1;
    }
    if (ret < 0) {
        return -1;
    }
    if (pf.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        errno = EIO;
        return -1;
    }
    if (!(pf.revents & (POLLIN | POLLPRI))) {
        errno = EAGAIN;
        return -1;
    }
    return ioctl(context->video_fd, VIDIOC_DQBUF, &context->vbuffer);
}

int v4l2_drm_dump_release(struct v4l2_drm_context* context) {
    return ioctl(context->video_fd, VIDIOC_QBUF, &context->vbuffer);
}
