#pragma once

#include <cstdlib>

#include "selfdrive/modeld/runners/runmodel.h"

struct rknn_runtime_state;

class RKNNModel : public RunModel {
public:
  RKNNModel(const char *path, float *output, size_t output_size, int runtime, bool use_extra = false);
  ~RKNNModel();
  void addRecurrent(float *state, int state_size);
  void addDesire(float *state, int state_size);
  void addTrafficConvention(float *state, int state_size);
  void addImage(float *image_buf, int buf_size);
  void addExtra(float *image_buf, int buf_size);
  void execute();
  bool needsNHWCInput() const override { return true; }
private:
  float *output;
  size_t output_size;
  rknn_runtime_state *runtime_state = nullptr;

  float *rnn_input_buf = NULL;
  int rnn_state_size;
  float *desire_input_buf = NULL;
  int desire_state_size;
  float *traffic_convention_input_buf = NULL;
  int traffic_convention_size;
  float *image_input_buf = NULL;
  int image_buf_size;
  float *extra_input_buf = NULL;
  int extra_buf_size;
  bool use_extra;
};
