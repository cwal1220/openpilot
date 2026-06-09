#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
cd "$ROOT"

export BASEDIR="$ROOT"
export PYTHONPATH="$ROOT:$ROOT/pyextra${PYTHONPATH:+:$PYTHONPATH}"
export PASSIVE="${PASSIVE:-0}"
export OPENPILOT_TARGET_ARCH="${OPENPILOT_TARGET_ARCH:-riscv64}"
export OPENPILOT_ACADOS_ARCH="${OPENPILOT_ACADOS_ARCH:-riscv64}"
export OPENPILOT_ACADOS_TOOL_ARCH="${OPENPILOT_ACADOS_TOOL_ARCH:-riscv64}"
export OPENPILOT_USE_PREGENERATED_ACADOS="${OPENPILOT_USE_PREGENERATED_ACADOS:-1}"
export ZMQ=1

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export QT_PLUGIN_PATH="${QT_PLUGIN_PATH:-/usr/lib/riscv64-linux-gnu/qt5/plugins}"
export QT_OPENGL="${QT_OPENGL:-software}"
export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export LD_LIBRARY_PATH="$ROOT/cereal:$ROOT/opendbc/can:$ROOT/selfdrive/common:$ROOT/common/transformations:$ROOT/rednose/helpers:$ROOT/selfdrive/locationd/models/generated:$ROOT/third_party/acados/riscv64/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

mkdir -p /data/log /data/params /data/media /dev/shm
ln -sfn "$ROOT" /data/openpilot
ln -sfn "$ROOT" /data/pythonpath
chmod 777 /dev/shm || true

exec "$@"
