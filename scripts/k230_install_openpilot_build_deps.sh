#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" != "0" ]]; then
  echo "run as root on the K230 Ubuntu board" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
PYCAPNP_SPEC="${K230_PYCAPNP_SPEC:-pycapnp==2.2.3}"
FUTURE_FSTRINGS_SPEC="${K230_FUTURE_FSTRINGS_SPEC:-future-fstrings==1.2.0}"

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  ca-certificates \
  capnproto \
  cython3 \
  gcc-arm-none-eabi \
  git \
  libavcodec-dev \
  libavformat-dev \
  libavutil-dev \
  libblas-dev \
  libbz2-dev \
  libcapnp-dev \
  libcurl4-openssl-dev \
  libegl1-mesa-dev \
  libeigen3-dev \
  libgl1-mesa-dev \
  libgles2-mesa-dev \
  libjpeg-dev \
  liblapack-dev \
  libqt5opengl5-dev \
  libqt5svg5-dev \
  libssl-dev \
  libswscale-dev \
  libsystemd-dev \
  libusb-1.0-0-dev \
  libyuv-dev \
  libzmq3-dev \
  ocl-icd-opencl-dev \
  opencl-headers \
  pkg-config \
  python3-cffi \
  python3-dev \
  python3-numpy \
  python3-pip \
  python3-pycryptodome \
  python3-requests \
  python3-scipy \
  python3-serial \
  python3-smbus2 \
  python3-setuptools \
  python3-sympy \
  python3-wheel \
  python3-zmq \
  qtbase5-dev \
  qtdeclarative5-dev \
  qtlocation5-dev \
  qtmultimedia5-dev \
  qtpositioning5-dev \
  rsync \
  scons \
  zlib1g-dev

python3 -c 'import capnp' 2>/dev/null || python3 -m pip install --break-system-packages --no-cache-dir "$PYCAPNP_SPEC"
python3 -c 'import future_fstrings' 2>/dev/null || python3 -m pip install --break-system-packages --no-cache-dir "$FUTURE_FSTRINGS_SPEC"

test -f /usr/include/libyuv.h
test -e /usr/lib/"$(dpkg-architecture -qDEB_HOST_MULTIARCH)"/libyuv.so
pkg-config --exists libavcodec
pkg-config --exists Qt5Core
pkg-config --exists Qt5Location
pkg-config --exists OpenCL
python3 -c 'import capnp'
