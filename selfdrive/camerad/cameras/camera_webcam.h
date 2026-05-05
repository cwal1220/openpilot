#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "selfdrive/camerad/cameras/camera_common.h"

#define FRAME_BUF_COUNT 4

typedef struct CameraState {
  int camera_num;
  int camera_id;
  int camera_index;
  std::string device_path;
  int fps;
  CameraInfo ci;

  float digital_gain = 0;
  CameraBuf buf;
  std::unique_ptr<cv::VideoCapture> cap;
  bool flip_x = false;
  bool flip_y = false;
} CameraState;

typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState driver_cam;

  SubMaster *sm = nullptr;
  PubMaster *pm = nullptr;
} MultiCameraState;
