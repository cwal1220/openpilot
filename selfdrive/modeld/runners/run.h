#pragma once

#include "runmodel.h"
#include "snpemodel.h"

#if defined(USE_THNEED)
#include "thneedmodel.h"
#endif

#if defined(USE_ONNX_MODEL)
#include "onnxmodel.h"
#endif

#if defined(USE_RKNN_MODEL)
#include "rknnmodel.h"
#endif
