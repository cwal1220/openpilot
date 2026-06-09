#include "selfdrive/modeld/runners/k230model.h"

#include <cstring>
#include <cmath>
#include <fstream>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>

#include "selfdrive/common/swaglog.h"

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;

namespace {

template <typename Shape>
size_t shape_count(const Shape &shape) {
  return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());
}

void require_float32(datatype_t datatype, const char *name) {
  if (datatype->typecode() != dt_float32) {
    throw std::runtime_error(std::string("K230 kmodel tensor is not float32: ") + name);
  }
}

constexpr float K230_RECURRENT_MAX_ABS = 100.0f;

}  // namespace

K230Model::K230Model(const char *path, float *output, size_t output_size, int, bool)
    : output_(output), output_size_(output_size) {
  LOGW("loading K230 kmodel %s", path);
  std::ifstream model(path, std::ios::binary);
  if (!model) {
    throw std::runtime_error(std::string("failed to open K230 kmodel: ") + path);
  }

  interp_.load_model(model).expect("invalid K230 kmodel");
  init_tensors();
}

void K230Model::init_tensors() {
  input_tensors_.reserve(interp_.inputs_size());
  for (size_t i = 0; i < interp_.inputs_size(); ++i) {
    const auto desc = interp_.input_desc(i);
    const auto shape = interp_.input_shape(i);
    require_float32(desc.datatype, "input");
    auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create K230 input tensor");
    interp_.input_tensor(i, tensor).expect("cannot set K230 input tensor");
    input_tensors_.push_back(tensor);
  }

  size_t total_output = 0;
  for (size_t i = 0; i < interp_.outputs_size(); ++i) {
    const auto desc = interp_.output_desc(i);
    const auto shape = interp_.output_shape(i);
    require_float32(desc.datatype, "output");
    total_output += shape_count(shape);
    auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create K230 output tensor");
    interp_.output_tensor(i, tensor).expect("cannot set K230 output tensor");
  }

  if (total_output != output_size_) {
    throw std::runtime_error("K230 kmodel output size mismatch: model=" + std::to_string(total_output) +
                             " openpilot=" + std::to_string(output_size_));
  }
}

size_t K230Model::input_count(size_t index) const {
  return shape_count(interp_.input_shape(index));
}

size_t K230Model::output_count(size_t index) const {
  return shape_count(interp_.output_shape(index));
}

void K230Model::write_input(size_t index, const float *data, size_t count, const char *name) {
  if (index >= input_tensors_.size()) {
    throw std::runtime_error(std::string("K230 kmodel missing input: ") + name);
  }
  if (data == nullptr) {
    throw std::runtime_error(std::string("K230 kmodel input not provided: ") + name);
  }

  const size_t expected = input_count(index);
  if (expected != count) {
    throw std::runtime_error(std::string("K230 kmodel input size mismatch for ") + name +
                             ": model=" + std::to_string(expected) +
                             " openpilot=" + std::to_string(count));
  }

  const size_t bytes = count * sizeof(float);
  {
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped_buffer = host_buffer.map(map_access_::map_write).unwrap();
    auto mapped = mapped_buffer.buffer();
    if (mapped.size() < bytes) {
      throw std::runtime_error(std::string("K230 kmodel input buffer too small for ") + name);
    }
    std::memcpy(reinterpret_cast<char *>(mapped.data()), data, bytes);
  }
  hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true).expect("K230 input sync failed");
}

void K230Model::write_zero_input_once(size_t index, size_t count, const char *name, bool &zeroed) {
  if (zeroed) {
    return;
  }

  if (index >= input_tensors_.size()) {
    throw std::runtime_error(std::string("K230 kmodel missing input: ") + name);
  }

  const size_t expected = input_count(index);
  if (expected != count) {
    throw std::runtime_error(std::string("K230 kmodel input size mismatch for ") + name +
                             ": model=" + std::to_string(expected) +
                             " openpilot=" + std::to_string(count));
  }

  const size_t bytes = count * sizeof(float);
  {
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped_buffer = host_buffer.map(map_access_::map_write).unwrap();
    auto mapped = mapped_buffer.buffer();
    if (mapped.size() < bytes) {
      throw std::runtime_error(std::string("K230 kmodel input buffer too small for ") + name);
    }
    std::memset(reinterpret_cast<char *>(mapped.data()), 0, bytes);
  }
  hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true).expect("K230 input zero sync failed");
  zeroed = true;
}

bool K230Model::sanitize_recurrent(const char *phase) {
  if (rnn_input_buf_ == nullptr || rnn_state_size_ <= 0) {
    return true;
  }

  int bad_index = -1;
  float bad_value = 0.0f;

  for (int i = 0; i < rnn_state_size_; ++i) {
    const float value = rnn_input_buf_[i];
    if (!std::isfinite(value) || std::fabs(value) > K230_RECURRENT_MAX_ABS) {
      bad_index = i;
      bad_value = value;
      break;
    }
  }

  if (bad_index >= 0) {
    std::memset(rnn_input_buf_, 0, static_cast<size_t>(rnn_state_size_) * sizeof(float));
    LOGE("resetting K230 recurrent state at %s: index=%d value=%f max_abs=%f",
         phase, bad_index, bad_value, K230_RECURRENT_MAX_ABS);
    return false;
  }

  return true;
}

void K230Model::addRecurrent(float *state, int state_size) {
  rnn_input_buf_ = state;
  rnn_state_size_ = state_size;
}

void K230Model::addDesire(float *state, int state_size) {
  desire_input_buf_ = state;
  desire_state_size_ = state_size;
}

void K230Model::addTrafficConvention(float *state, int state_size) {
  traffic_convention_input_buf_ = state;
  traffic_convention_size_ = state_size;
}

void K230Model::addImage(float *image_buf, int buf_size) {
  image_input_buf_ = image_buf;
  image_buf_size_ = buf_size;
}

void K230Model::addExtra(float *image_buf, int buf_size) {
  extra_input_buf_ = image_buf;
  extra_buf_size_ = buf_size;
  if (image_buf != nullptr) {
    extra_input_zeroed_ = false;
  }
}

void K230Model::execute() {
  write_input(0, image_input_buf_, image_buf_size_, "input_imgs");
  if (extra_input_buf_ != nullptr) {
    write_input(1, extra_input_buf_, extra_buf_size_, "big_input_imgs");
  } else {
    write_zero_input_once(1, extra_buf_size_, "big_input_imgs", extra_input_zeroed_);
  }
  write_input(2, desire_input_buf_, desire_state_size_, "desire");
  write_input(3, traffic_convention_input_buf_, traffic_convention_size_, "traffic_convention");
  sanitize_recurrent("input");
  write_input(4, rnn_input_buf_, rnn_state_size_, "initial_state");

  interp_.run().expect("K230 kmodel run failed");

  size_t offset = 0;
  for (size_t i = 0; i < interp_.outputs_size(); ++i) {
    auto out = interp_.output_tensor(i).expect("cannot get K230 output tensor");
    hrt::sync(out, sync_op_t::sync_invalidate, true).expect("K230 output sync failed");
    const size_t count = output_count(i);
    const size_t bytes = count * sizeof(float);
    {
      auto host_buffer = out.impl()->to_host().unwrap()->buffer().as_host().unwrap();
      auto mapped_buffer = host_buffer.map(map_access_::map_read).unwrap();
      auto mapped = mapped_buffer.buffer();
      if (offset + count > output_size_ || mapped.size() < bytes) {
        throw std::runtime_error("K230 kmodel output buffer size mismatch");
      }
      std::memcpy(output_ + offset, reinterpret_cast<const char *>(mapped.data()), bytes);
    }
    offset += count;
  }
  sanitize_recurrent("output");
}
