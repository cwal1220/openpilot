#include "selfdrive/modeld/runners/rknnmodel.h"

#include <cassert>

#include "selfdrive/modeld/runners/rknn_runtime.h"
#include "selfdrive/common/swaglog.h"

RKNNModel::RKNNModel(const char *path, float *_output, size_t _output_size, int runtime, bool _use_extra) {
  LOGD("loading model %s", path);
  (void)runtime;

  output = _output;
  output_size = _output_size;
  use_extra = _use_extra;
  runtime_state = rknn_runtime_create(path, use_extra, output_size);
  assert(runtime_state != nullptr);
}

RKNNModel::~RKNNModel() {
  rknn_runtime_destroy(runtime_state);
}

void RKNNModel::addRecurrent(float *state, int state_size) {
  rnn_input_buf = state;
  rnn_state_size = state_size;
}

void RKNNModel::addDesire(float *state, int state_size) {
  desire_input_buf = state;
  desire_state_size = state_size;
}

void RKNNModel::addTrafficConvention(float *state, int state_size) {
  traffic_convention_input_buf = state;
  traffic_convention_size = state_size;
}

void RKNNModel::addImage(float *image_buf, int buf_size) {
  image_input_buf = image_buf;
  image_buf_size = buf_size;
}

void RKNNModel::addExtra(float *image_buf, int buf_size) {
  extra_input_buf = image_buf;
  extra_buf_size = buf_size;
}

void RKNNModel::execute() {
  int ret = rknn_runtime_execute(runtime_state,
                                 image_input_buf, image_buf_size,
                                 extra_input_buf, extra_buf_size,
                                 desire_input_buf, desire_state_size,
                                 traffic_convention_input_buf, traffic_convention_size,
                                 rnn_input_buf, rnn_state_size,
                                 output, output_size);
  if (ret != 0) {
    LOGE("rknn_runtime_execute failed: %d", ret);
  }
}
