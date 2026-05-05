#include "selfdrive/modeld/runners/rknn_runtime.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dlfcn.h>

// Minimal RKNN C API declarations derived from the official RKNN runtime header.
typedef uint64_t rknn_context;

#define RKNN_SUCC 0
#define RKNN_QUERY_IN_OUT_NUM 0
#define RKNN_QUERY_INPUT_ATTR 1
#define RKNN_QUERY_OUTPUT_ATTR 2
#define RKNN_QUERY_SDK_VERSION 5

typedef enum {
  RKNN_TENSOR_FLOAT32 = 0,
} rknn_tensor_type;

typedef enum {
  RKNN_TENSOR_QNT_NONE = 0,
  RKNN_TENSOR_QNT_DFP = 1,
  RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC = 2,
} rknn_tensor_qnt_type;

typedef enum {
  RKNN_TENSOR_NCHW = 0,
  RKNN_TENSOR_NHWC = 1,
  RKNN_TENSOR_NC1HWC2 = 2,
  RKNN_TENSOR_UNDEFINED = 3,
} rknn_tensor_format;

typedef enum {
  RKNN_NPU_CORE_AUTO = 0,
  RKNN_NPU_CORE_0 = 1,
  RKNN_NPU_CORE_1 = 2,
  RKNN_NPU_CORE_2 = 4,
  RKNN_NPU_CORE_0_1 = 3,
  RKNN_NPU_CORE_0_1_2 = 7,
  RKNN_NPU_CORE_ALL = 0xffff,
  RKNN_NPU_CORE_UNDEFINED = 0x10000,
} rknn_core_mask;

typedef struct {
  uint32_t n_input;
  uint32_t n_output;
} rknn_input_output_num;

typedef struct {
  uint32_t index;
  uint32_t n_dims;
  uint32_t dims[16];
  char name[256];
  uint32_t n_elems;
  uint32_t size;
  rknn_tensor_format fmt;
  rknn_tensor_type type;
  rknn_tensor_qnt_type qnt_type;
  int8_t fl;
  int32_t zp;
  float scale;
  uint32_t w_stride;
  uint32_t size_with_stride;
  uint8_t pass_through;
  uint32_t h_stride;
} rknn_tensor_attr;

typedef struct {
  char api_version[256];
  char drv_version[256];
} rknn_sdk_version;

typedef struct {
  uint32_t index;
  void *buf;
  uint32_t size;
  uint8_t pass_through;
  rknn_tensor_type type;
  rknn_tensor_format fmt;
} rknn_input;

typedef struct {
  uint8_t want_float;
  uint8_t is_prealloc;
  uint32_t index;
  void *buf;
  uint32_t size;
} rknn_output;

typedef int (*rknn_init_fn)(rknn_context *context, void *model, uint32_t size, uint32_t flag, void *extend);
typedef int (*rknn_destroy_fn)(rknn_context context);
typedef int (*rknn_query_fn)(rknn_context context, int cmd, void *info, uint32_t size);
typedef int (*rknn_inputs_set_fn)(rknn_context context, uint32_t n_inputs, rknn_input inputs[]);
typedef int (*rknn_set_core_mask_fn)(rknn_context context, rknn_core_mask core_mask);
typedef int (*rknn_run_fn)(rknn_context context, void *extend);
typedef int (*rknn_outputs_get_fn)(rknn_context context, uint32_t n_outputs, rknn_output outputs[], void *extend);
typedef int (*rknn_outputs_release_fn)(rknn_context context, uint32_t n_outputs, rknn_output outputs[]);

typedef struct {
  void *handle;
  rknn_init_fn init;
  rknn_destroy_fn destroy;
  rknn_query_fn query;
  rknn_inputs_set_fn inputs_set;
  rknn_set_core_mask_fn set_core_mask;
  rknn_run_fn run;
  rknn_outputs_get_fn outputs_get;
  rknn_outputs_release_fn outputs_release;
} rknn_api;

struct rknn_runtime_state {
  rknn_api api;
  rknn_context ctx;
  bool use_extra;
  size_t output_size;
  rknn_input_output_num io_num;
  rknn_tensor_attr input_attrs[5];
  rknn_tensor_attr output_attrs[4];
  uint8_t *image1_zeros;
  size_t image1_size;
};

static void rknn_log_tensor_attr(const char *label, const rknn_tensor_attr *attr) {
  fprintf(stderr, "rknn %s[%u] name=%s dims=", label, attr->index, attr->name);
  for (uint32_t i = 0; i < attr->n_dims; ++i) {
    fprintf(stderr, "%u%s", attr->dims[i], (i + 1 == attr->n_dims) ? "" : "x");
  }
  fprintf(stderr, " fmt=%d type=%d qnt=%d n_elems=%u size=%u\n",
          attr->fmt, attr->type, attr->qnt_type, attr->n_elems, attr->size);
}

static int load_symbol(void *handle, const char *name, void **fn_ptr) {
  *fn_ptr = dlsym(handle, name);
  if (*fn_ptr == NULL) {
    fprintf(stderr, "rknn dlsym failed for %s: %s\n", name, dlerror());
    return -1;
  }
  return 0;
}

static int rknn_api_open(rknn_api *api) {
  memset(api, 0, sizeof(*api));
  api->handle = dlopen("librknnrt.so", RTLD_NOW | RTLD_LOCAL);
  if (api->handle == NULL) {
    fprintf(stderr, "dlopen librknnrt.so failed: %s\n", dlerror());
    return -1;
  }

  if (load_symbol(api->handle, "rknn_init", (void **)&api->init) != 0 ||
      load_symbol(api->handle, "rknn_destroy", (void **)&api->destroy) != 0 ||
      load_symbol(api->handle, "rknn_query", (void **)&api->query) != 0 ||
      load_symbol(api->handle, "rknn_inputs_set", (void **)&api->inputs_set) != 0 ||
      load_symbol(api->handle, "rknn_set_core_mask", (void **)&api->set_core_mask) != 0 ||
      load_symbol(api->handle, "rknn_run", (void **)&api->run) != 0 ||
      load_symbol(api->handle, "rknn_outputs_get", (void **)&api->outputs_get) != 0 ||
      load_symbol(api->handle, "rknn_outputs_release", (void **)&api->outputs_release) != 0) {
    dlclose(api->handle);
    memset(api, 0, sizeof(*api));
    return -1;
  }

  return 0;
}

static void rknn_api_close(rknn_api *api) {
  if (api->handle != NULL) {
    dlclose(api->handle);
  }
  memset(api, 0, sizeof(*api));
}

static rknn_core_mask get_core_mask_from_env(void) {
  const char *name = getenv("RKNN_CORE_MASK");
  if (name == NULL || strcmp(name, "") == 0) return RKNN_NPU_CORE_0_1_2;
  if (strcmp(name, "NPU_CORE_AUTO") == 0) return RKNN_NPU_CORE_AUTO;
  if (strcmp(name, "NPU_CORE_0") == 0) return RKNN_NPU_CORE_0;
  if (strcmp(name, "NPU_CORE_1") == 0) return RKNN_NPU_CORE_1;
  if (strcmp(name, "NPU_CORE_2") == 0) return RKNN_NPU_CORE_2;
  if (strcmp(name, "NPU_CORE_0_1") == 0) return RKNN_NPU_CORE_0_1;
  if (strcmp(name, "NPU_CORE_0_1_2") == 0) return RKNN_NPU_CORE_0_1_2;
  if (strcmp(name, "NPU_CORE_ALL") == 0) return RKNN_NPU_CORE_ALL;
  return RKNN_NPU_CORE_0_1_2;
}

rknn_runtime_state *rknn_runtime_create(const char *model_path, bool use_extra, size_t output_size) {
  rknn_runtime_state *state = (rknn_runtime_state *)calloc(1, sizeof(*state));
  if (state == NULL) return NULL;

  state->use_extra = use_extra;
  state->output_size = output_size;

  if (rknn_api_open(&state->api) != 0) goto fail;

  int ret = state->api.init(&state->ctx, (void *)model_path, 0, 0, NULL);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_init failed: %d (%s)\n", ret, model_path);
    goto fail;
  }

  rknn_core_mask core_mask = get_core_mask_from_env();
  ret = state->api.set_core_mask(state->ctx, core_mask);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_set_core_mask failed: %d\n", ret);
  }

  rknn_sdk_version sdk_version;
  memset(&sdk_version, 0, sizeof(sdk_version));
  ret = state->api.query(state->ctx, RKNN_QUERY_SDK_VERSION, &sdk_version, sizeof(sdk_version));
  if (ret == RKNN_SUCC) {
    fprintf(stderr, "rknn sdk api=%s drv=%s\n", sdk_version.api_version, sdk_version.drv_version);
  }

  ret = state->api.query(state->ctx, RKNN_QUERY_IN_OUT_NUM, &state->io_num, sizeof(state->io_num));
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_query IN_OUT_NUM failed: %d\n", ret);
    goto fail;
  }

  if (state->io_num.n_input < 5 || state->io_num.n_output < 1) {
    fprintf(stderr, "unexpected rknn io count: inputs=%u outputs=%u\n", state->io_num.n_input, state->io_num.n_output);
    goto fail;
  }

  for (uint32_t i = 0; i < 5; ++i) {
    memset(&state->input_attrs[i], 0, sizeof(state->input_attrs[i]));
    state->input_attrs[i].index = i;
    ret = state->api.query(state->ctx, RKNN_QUERY_INPUT_ATTR, &state->input_attrs[i], sizeof(state->input_attrs[i]));
    if (ret != RKNN_SUCC) {
      fprintf(stderr, "rknn_query INPUT_ATTR[%u] failed: %d\n", i, ret);
      goto fail;
    }
    rknn_log_tensor_attr("input", &state->input_attrs[i]);
  }

  for (uint32_t i = 0; i < state->io_num.n_output && i < 4; ++i) {
    memset(&state->output_attrs[i], 0, sizeof(state->output_attrs[i]));
    state->output_attrs[i].index = i;
    ret = state->api.query(state->ctx, RKNN_QUERY_OUTPUT_ATTR, &state->output_attrs[i], sizeof(state->output_attrs[i]));
    if (ret != RKNN_SUCC) {
      fprintf(stderr, "rknn_query OUTPUT_ATTR[%u] failed: %d\n", i, ret);
      goto fail;
    }
    rknn_log_tensor_attr("output", &state->output_attrs[i]);
  }

  state->image1_size = state->input_attrs[1].size;
  state->image1_zeros = (uint8_t *)calloc(1, state->image1_size);
  if (state->image1_zeros == NULL) {
    fprintf(stderr, "calloc failed for rknn extra image scratch buffer\n");
    goto fail;
  }

  return state;

fail:
  rknn_runtime_destroy(state);
  return NULL;
}

void rknn_runtime_destroy(rknn_runtime_state *state) {
  if (state == NULL) return;
  free(state->image1_zeros);
  if (state->ctx != 0 && state->api.destroy != NULL) {
    state->api.destroy(state->ctx);
  }
  rknn_api_close(&state->api);
  free(state);
}

int rknn_runtime_execute(rknn_runtime_state *state,
                         float *image_input_buf, int image_buf_size,
                         float *extra_input_buf, int extra_buf_size,
                         float *desire_input_buf, int desire_state_size,
                         float *traffic_convention_input_buf, int traffic_convention_size,
                         float *rnn_input_buf, int rnn_state_size,
                         float *output, size_t output_size) {
  if (state == NULL || image_input_buf == NULL || output == NULL) return -1;

  rknn_input inputs[5];
  memset(inputs, 0, sizeof(inputs));

  inputs[0].index = 0;
  inputs[0].buf = image_input_buf;
  inputs[0].size = (uint32_t)(image_buf_size * sizeof(float));
  inputs[0].pass_through = 0;
  inputs[0].type = RKNN_TENSOR_FLOAT32;
  inputs[0].fmt = RKNN_TENSOR_NHWC;

  inputs[1].index = 1;
  inputs[1].buf = (extra_input_buf != NULL) ? (void *)extra_input_buf : (void *)state->image1_zeros;
  inputs[1].size = (extra_input_buf != NULL) ? (uint32_t)(extra_buf_size * sizeof(float)) : (uint32_t)state->image1_size;
  inputs[1].pass_through = 0;
  inputs[1].type = (extra_input_buf != NULL) ? RKNN_TENSOR_FLOAT32 : state->input_attrs[1].type;
  inputs[1].fmt = state->input_attrs[1].fmt;

  inputs[2].index = 2;
  inputs[2].buf = desire_input_buf;
  inputs[2].size = (uint32_t)(desire_state_size * sizeof(float));
  inputs[2].pass_through = 0;
  inputs[2].type = RKNN_TENSOR_FLOAT32;
  inputs[2].fmt = state->input_attrs[2].fmt;

  inputs[3].index = 3;
  inputs[3].buf = traffic_convention_input_buf;
  inputs[3].size = (uint32_t)(traffic_convention_size * sizeof(float));
  inputs[3].pass_through = 0;
  inputs[3].type = RKNN_TENSOR_FLOAT32;
  inputs[3].fmt = state->input_attrs[3].fmt;

  inputs[4].index = 4;
  inputs[4].buf = rnn_input_buf;
  inputs[4].size = (uint32_t)(rnn_state_size * sizeof(float));
  inputs[4].pass_through = 0;
  inputs[4].type = RKNN_TENSOR_FLOAT32;
  inputs[4].fmt = state->input_attrs[4].fmt;

  int ret = state->api.inputs_set(state->ctx, 5, inputs);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_inputs_set failed: %d image_size=%d extra_size=%d\n", ret, image_buf_size, extra_buf_size);
    return ret;
  }

  ret = state->api.run(state->ctx, NULL);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_run failed: %d\n", ret);
    return ret;
  }

  rknn_output outputs[4];
  memset(outputs, 0, sizeof(outputs));
  for (uint32_t i = 0; i < state->io_num.n_output && i < 4; ++i) {
    outputs[i].index = i;
    outputs[i].want_float = 1;
    outputs[i].is_prealloc = 0;
  }

  ret = state->api.outputs_get(state->ctx, state->io_num.n_output, outputs, NULL);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "rknn_outputs_get failed: %d\n", ret);
    return ret;
  }

  size_t out_floats_written = 0;
  for (uint32_t i = 0; i < state->io_num.n_output && i < 4; ++i) {
    size_t n_elems = state->output_attrs[i].n_elems;
    size_t remaining = (output_size > out_floats_written) ? (output_size - out_floats_written) : 0;
    size_t copy_elems = (n_elems < remaining) ? n_elems : remaining;
    if (copy_elems > 0 && outputs[i].buf != NULL) {
      memcpy(output + out_floats_written, outputs[i].buf, copy_elems * sizeof(float));
      out_floats_written += copy_elems;
    }
  }
  if (out_floats_written < output_size) {
    memset(output + out_floats_written, 0, (output_size - out_floats_written) * sizeof(float));
  }

  state->api.outputs_release(state->ctx, state->io_num.n_output, outputs);
  return RKNN_SUCC;
}
