#!/bin/bash

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$(cd "$DIR/../" && pwd)"

APT_PACKAGES=(
  build-essential
  binutils-arm-none-eabi
  clang
  cmake
  gcc-arm-none-eabi
  git
  git-lfs
  curl
  ca-certificates
  scons
  capnproto
  libcapnp-dev
  libeigen3-dev
  libegl1-mesa-dev
  libgles2-mesa-dev
  libjpeg-dev
  libopencv-dev
  libusb-1.0-0-dev
  libyuv-dev
  libzmq3-dev
  mesa-opencl-icd
  ocl-icd-opencl-dev
  opencl-headers
  clinfo
  pocl-opencl-icd
  python3-dev
  python3-jinja2
  python3-numpy
  python3-pip
  cython3
)

OPTIONAL_APT_PACKAGES=(
  fonts-nanum
  fonts-noto-cjk
)

usage() {
  cat <<EOF
Usage: tools/orangepi_setup.sh [--with-korean-fonts]

Installs the Orange Pi 5 build/runtime packages used during the
generic Linux ARM64 bring-up work, then installs Python packages
from tools/orangepi5_python_requirements.txt and creates the basic
openpilot runtime directories.
EOF
}

WITH_KOREAN_FONTS=0
for arg in "$@"; do
  case "$arg" in
    --with-korean-fonts)
      WITH_KOREAN_FONTS=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage
      exit 1
      ;;
  esac
done

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This script currently supports apt-based distributions only." >&2
  exit 1
fi

echo "[orangepi_setup] installing apt packages"
sudo apt-get update
sudo apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"

if command -v udevadm >/dev/null 2>&1; then
  echo "[orangepi_setup] installing panda udev rule"
  tmp_rule="$(mktemp)"
  cat >"$tmp_rule" <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddcc", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddee", MODE="0660", GROUP="plugdev"
EOF
  sudo install -m 0644 "$tmp_rule" /etc/udev/rules.d/11-panda.rules
  rm -f "$tmp_rule"
  sudo udevadm control --reload-rules || true
  sudo udevadm trigger --attr-match=idVendor=bbaa || true
fi

echo "[orangepi_setup] creating openpilot runtime directories"
sudo mkdir -p /data/params/d /data/log /data/media/0/realdata
sudo chown -R "$(id -u):$(id -g)" /data

if [ "$WITH_KOREAN_FONTS" -eq 1 ]; then
  echo "[orangepi_setup] installing optional Korean fonts"
  sudo apt-get install -y --no-install-recommends "${OPTIONAL_APT_PACKAGES[@]}"
fi

echo "[orangepi_setup] installing python packages"
python3 -m pip install --user --break-system-packages -r "$DIR/orangepi5_python_requirements.txt"

echo
echo "[orangepi_setup] done"
echo "Recommended checks:"
echo "  clinfo | sed -n '1,40p'"
echo "  python3 -c 'import capnp, zmq, usb1, serial, sentry_sdk, tqdm, crcmod, cffi, onnx, future_fstrings'"
echo "  cd $ROOT && OPENPILOT_BUILD_PLATFORM=linux_generic USE_WEBCAM=1 NOSENSOR=1 scons -j$(nproc) selfdrive/camerad/camerad selfdrive/modeld/_modeld selfdrive/ui/_ui selfdrive/clocksd/clocksd selfdrive/proclogd/proclogd selfdrive/locationd/locationd common/clock.so common/params_pyx.so common/kalman/simple_kalman_impl.so common/transformations/transformations.so cereal/messaging/messaging_pyx.so cereal/visionipc/visionipc_pyx.so opendbc/can/parser_pyx.so opendbc/can/packer_pyx.so rednose/helpers/ekf_sym_pyx.so selfdrive/boardd/boardd_api_impl.so selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so selfdrive/controls/lib/cluster/libfastcluster.so"
echo "  cd $ROOT && OPENPILOT_BUILD_PLATFORM=linux_generic scons -j$(nproc) selfdrive/boardd/boardd panda/board/obj/panda.bin.signed panda/board/obj/panda_h7.bin.signed"
