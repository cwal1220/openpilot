#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rknn_runtime_state rknn_runtime_state;

rknn_runtime_state *rknn_runtime_create(const char *model_path, bool use_extra, size_t output_size);
void rknn_runtime_destroy(rknn_runtime_state *state);
int rknn_runtime_execute(rknn_runtime_state *state,
                         float *image_input_buf, int image_buf_size,
                         float *extra_input_buf, int extra_buf_size,
                         float *desire_input_buf, int desire_state_size,
                         float *traffic_convention_input_buf, int traffic_convention_size,
                         float *rnn_input_buf, int rnn_state_size,
                         float *output, size_t output_size);

#ifdef __cplusplus
}
#endif
