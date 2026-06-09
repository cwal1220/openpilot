#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${K230_OPENPILOT_BASE_IMAGE:-supercombo-k230-toolchain:24.04}"
ACADOS_TAG="${K230_ACADOS_TAG:-v0.1.8}"
SRC="${K230_ACADOS_SRC:-$ROOT/build/k230_acados_${ACADOS_TAG#v}}"
OUT="$ROOT/third_party/acados/riscv64/lib"

if [[ ! -d "$SRC/.git" ]]; then
  rm -rf "$SRC"
  git clone \
    --branch "$ACADOS_TAG" \
    --depth 1 \
    --recurse-submodules \
    --shallow-submodules \
    https://github.com/acados/acados.git \
    "$SRC"
else
  git -C "$SRC" fetch --depth 1 origin "refs/tags/$ACADOS_TAG:refs/tags/$ACADOS_TAG"
  git -C "$SRC" checkout --detach "$ACADOS_TAG"
  git -C "$SRC" submodule update --init --recursive --depth 1
fi

rm -rf "$SRC/build-riscv64" "$SRC/install-riscv64"

docker run --rm \
  -v "$SRC:/src" \
  -w /src \
  "$IMAGE" \
  bash -lc 'set -euo pipefail
cmake -S . -B build-riscv64 \
  -DCMAKE_SYSTEM_NAME=Linux \
  -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
  -DCMAKE_C_COMPILER=riscv64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=riscv64-linux-gnu-g++ \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DBLASFEO_TARGET=GENERIC \
  -DHPIPM_TARGET=GENERIC \
  -DLA=HIGH_PERFORMANCE \
  -DACADOS_WITH_QPOASES=ON \
  -DACADOS_WITH_OPENMP=OFF \
  -DACADOS_INSTALL_DIR=/src/install-riscv64
cmake --build build-riscv64 -j"$(nproc)"
cmake --install build-riscv64'

mkdir -p "$OUT"
cp "$SRC/install-riscv64/lib/libacados.so" "$OUT/"
cp "$SRC/install-riscv64/lib/libblasfeo.so" "$OUT/"
cp "$SRC/install-riscv64/lib/libhpipm.so" "$OUT/"
cp "$SRC/install-riscv64/lib/libqpOASES_e.so.3.1" "$OUT/"
ln -sfn libqpOASES_e.so.3.1 "$OUT/libqpOASES_e.so"

file "$OUT"/libacados.so "$OUT"/libblasfeo.so "$OUT"/libhpipm.so "$OUT"/libqpOASES_e.so.3.1
