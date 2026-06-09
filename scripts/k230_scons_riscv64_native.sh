#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export OPENPILOT_TARGET_ARCH="${OPENPILOT_TARGET_ARCH:-riscv64}"
export OPENPILOT_ACADOS_ARCH="${OPENPILOT_ACADOS_ARCH:-riscv64}"
export OPENPILOT_ACADOS_TOOL_ARCH="${OPENPILOT_ACADOS_TOOL_ARCH:-riscv64}"
export OPENPILOT_USE_PREGENERATED_ACADOS="${OPENPILOT_USE_PREGENERATED_ACADOS:-1}"
export OPENPILOT_PYTHON_INCLUDE="${OPENPILOT_PYTHON_INCLUDE:-/usr/include/python3.12}"
export OPENPILOT_PYTHON_LIB="${OPENPILOT_PYTHON_LIB:-/usr/lib/riscv64-linux-gnu}"
export OPENPILOT_PYTHON_LIBNAME="${OPENPILOT_PYTHON_LIBNAME:-python3.12}"
export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"
export AR="${AR:-ar}"
export RANLIB="${RANLIB:-ranlib}"
export CYTHON="${CYTHON:-cython}"

JOBS="${JOBS:-1}"
exec scons -j"$JOBS" --no-thneed "$@"
