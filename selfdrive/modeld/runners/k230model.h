#pragma once

#include <cstddef>
#include <vector>

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

#include "selfdrive/modeld/runners/runmodel.h"

class K230Model : public RunModel {
public:
  K230Model(const char *path, float *output, size_t output_size, int runtime, bool use_extra = false);

  void addRecurrent(float *state, int state_size) override;
  void addDesire(float *state, int state_size) override;
  void addTrafficConvention(float *state, int state_size) override;
  void addImage(float *image_buf, int buf_size) override;
  void addExtra(float *image_buf, int buf_size) override;
  void execute() override;

private:
  void init_tensors();
  void write_input(size_t index, const float *data, size_t count, const char *name);
  void write_zero_input_once(size_t index, size_t count, const char *name, bool &zeroed);
  bool sanitize_recurrent(const char *phase);
  size_t input_count(size_t index) const;
  size_t output_count(size_t index) const;

  nncase::runtime::interpreter interp_;
  std::vector<nncase::runtime::runtime_tensor> input_tensors_;

  float *output_ = nullptr;
  size_t output_size_ = 0;

  float *image_input_buf_ = nullptr;
  int image_buf_size_ = 0;
  float *extra_input_buf_ = nullptr;
  int extra_buf_size_ = 0;
  bool extra_input_zeroed_ = false;
  float *desire_input_buf_ = nullptr;
  int desire_state_size_ = 0;
  float *traffic_convention_input_buf_ = nullptr;
  int traffic_convention_size_ = 0;
  float *rnn_input_buf_ = nullptr;
  int rnn_state_size_ = 0;
};
