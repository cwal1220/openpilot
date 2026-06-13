#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="${K230_OPENPILOT_BASE_IMAGE:-supercombo-k230-toolchain:24.04}"
IMAGE="${K230_OPENPILOT_TOOLCHAIN_IMAGE:-openpilot-riscv64-toolchain:24.04}"
SYSROOT="${K230_SYSROOT:-$ROOT/k230_sysroot}"
NNCASE_DIR="${K230_NNCASE_DIR:-$ROOT/third_party/nncase_k230}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"
CASADI_VERSION="${K230_CASADI_VERSION:-3.7.1}"
PYCAPNP_SPEC="${K230_PYCAPNP_SPEC:-pycapnp==2.2.3}"
FUTURE_FSTRINGS_SPEC="${K230_FUTURE_FSTRINGS_SPEC:-future-fstrings==1.2.0}"
RISCV_OPT_FLAGS="${OPENPILOT_RISCV_OPT_FLAGS--march=rv64gcv -mabi=lp64d}"

if [[ ! -d "$SYSROOT/usr/lib/riscv64-linux-gnu" ]]; then
  echo "missing K230 sysroot: $SYSROOT" >&2
  echo "set K230_SYSROOT to a synced Ubuntu riscv64 board sysroot" >&2
  exit 1
fi

if [[ ! -f "$NNCASE_DIR/lib/libNncase.Runtime.Native.a" ]]; then
  K230_NNCASE_DIR="$NNCASE_DIR" "$ROOT/scripts/k230_install_nncase_runtime.sh"
fi

need_image_build=0
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  need_image_build=1
elif [[ "${K230_REBUILD_OPENPILOT_TOOLCHAIN:-0}" == "1" ]]; then
  need_image_build=1
elif ! docker run -i --rm "$IMAGE" bash -lc 'command -v arm-none-eabi-gcc >/dev/null && python3 -' <<'PY' >/dev/null 2>&1
import casadi
import capnp
import cffi
import future_fstrings
import jinja2
import numpy
import requests
import scipy
import serial
import smbus2
import sympy
import zmq
try:
  import Crypto
except ModuleNotFoundError:
  import Cryptodome
PY
then
  need_image_build=1
fi

if [[ "$need_image_build" == "1" ]]; then
  docker build \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg CASADI_VERSION="$CASADI_VERSION" \
    --build-arg PYCAPNP_SPEC="$PYCAPNP_SPEC" \
    --build-arg FUTURE_FSTRINGS_SPEC="$FUTURE_FSTRINGS_SPEC" \
    -t "$IMAGE" \
    -f - "$ROOT" <<'DOCKERFILE'
ARG BASE_IMAGE
ARG CASADI_VERSION
ARG PYCAPNP_SPEC
ARG FUTURE_FSTRINGS_SPEC
FROM ${BASE_IMAGE}
ARG CASADI_VERSION
ARG PYCAPNP_SPEC
ARG FUTURE_FSTRINGS_SPEC
RUN apt-get update && apt-get install -y --no-install-recommends \
    capnproto \
    cython3 \
    gcc-arm-none-eabi \
    libcapnp-dev \
    pkg-config \
    python3 \
    python3-cffi \
    python3-dev \
    python3-jinja2 \
    python3-numpy \
    python3-pip \
    python3-pycryptodome \
    python3-requests \
    python3-scipy \
    python3-serial \
    python3-smbus2 \
    python3-sympy \
    python3-zmq \
    qt5-qmake \
    qtbase5-dev-tools \
    scons \
  && python3 -m pip install --break-system-packages --no-cache-dir "casadi==${CASADI_VERSION}" "${PYCAPNP_SPEC}" "${FUTURE_FSTRINGS_SPEC}" \
  && rm -rf /var/lib/apt/lists/*
DOCKERFILE
fi

docker run --rm \
  -e OPENPILOT_RISCV_OPT_FLAGS="$RISCV_OPT_FLAGS" \
  -v "$ROOT:/work/openpilot" \
  -v "$SYSROOT:/work/sysroot:ro" \
  -w /work/openpilot \
  "$IMAGE" \
  bash -lc "set -euo pipefail
export OPENPILOT_TARGET_ARCH=riscv64
export OPENPILOT_SYSROOT=/work/sysroot
export OPENPILOT_LINUX_MULTIARCH=riscv64-linux-gnu
export OPENPILOT_ACADOS_ARCH=riscv64
export OPENPILOT_ACADOS_TOOL_ARCH=larch64
export OPENPILOT_PYTHON_INCLUDE=/work/sysroot/usr/include/python3.12
export OPENPILOT_PYTHON_LIB=/work/sysroot/usr/lib/riscv64-linux-gnu
export OPENPILOT_PYTHON_LIBNAME=python3.12
export CC=riscv64-linux-gnu-gcc
export CXX=riscv64-linux-gnu-g++
export AR=riscv64-linux-gnu-ar
export RANLIB=riscv64-linux-gnu-ranlib
export CYTHON=cython
export PKG_CONFIG_SYSROOT_DIR=/work/sysroot
export PKG_CONFIG_LIBDIR=/work/sysroot/usr/lib/riscv64-linux-gnu/pkgconfig:/work/sysroot/usr/share/pkgconfig
scons -j$JOBS --no-thneed \"\$@\"
" k230-cross "$@"
