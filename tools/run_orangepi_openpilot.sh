#!/bin/bash

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$(cd "$DIR/../" && pwd)"

cd "$ROOT"

export PYTHONPATH="$ROOT"
export OPENPILOT_BUILD_PLATFORM=linux_generic
REAL_CAR="${ORANGEPI_REAL_CAR:-0}"

# Clear stale block list inherited from an interactive shell.
unset BLOCK

# Try to attach the UI to the active desktop session when launched over SSH.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
fi

export PASSIVE=1
export USE_WEBCAM=1

if [ "$REAL_CAR" = "1" ]; then
  # Real-car mode: let Panda/CAN and vehicle state drive ignition and controls.
  unset FORCE_IGNITION_ON
  unset NOSENSOR
  unset NOBOARD
  unset DISABLE_DRIVER_MONITORING
else
  # Bench bring-up defaults.
  export FORCE_IGNITION_ON=1
  export NOSENSOR=1
  export DISABLE_DRIVER_MONITORING=1
fi

# Current Orange Pi webcam defaults
export ROADCAM_DEV="${ROADCAM_DEV:-/dev/video0}"
export ROADCAM_FOURCC="${ROADCAM_FOURCC:-MJPG}"
export ROADCAM_WIDTH="${ROADCAM_WIDTH:-1280}"
export ROADCAM_HEIGHT="${ROADCAM_HEIGHT:-720}"

# Calibrated webcam intrinsics for HD WEBCAM Web Camera @ 1280x720
export WEBCAM_FX="${WEBCAM_FX:-966.530441}"
export WEBCAM_FY="${WEBCAM_FY:-965.972542}"
export WEBCAM_CX="${WEBCAM_CX:-578.967537}"
export WEBCAM_CY="${WEBCAM_CY:-273.096698}"

# Optional: exclude UI and sound if you want a headless bench run.
# export BLOCK=ui,soundd

# Optional: run without Panda connected.
# export NOBOARD=1

# Optional: 1080p calibrated intrinsics for the same webcam model.
# export ROADCAM_WIDTH=1920
# export ROADCAM_HEIGHT=1080
# export WEBCAM_FX=1461.953014
# export WEBCAM_FY=1470.545121
# export WEBCAM_CX=867.977124
# export WEBCAM_CY=405.202143

# Optional: override webcam node if enumeration changes.
# export ROADCAM_DEV=/dev/video1

echo "[run_orangepi_openpilot] starting openpilot from $ROOT"
echo "[run_orangepi_openpilot] mode: $([ "$REAL_CAR" = "1" ] && echo real-car || echo bench)"
echo "[run_orangepi_openpilot] camera: $ROADCAM_DEV ${ROADCAM_WIDTH}x${ROADCAM_HEIGHT} $ROADCAM_FOURCC"

exec python3 selfdrive/manager/manager.py
