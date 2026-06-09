#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NNCASE_VERSION="${K230_NNCASE_VERSION:-2.11.0}"
OUT="${K230_NNCASE_DIR:-$ROOT/third_party/nncase_k230}"
FORCE="${K230_FORCE_NNCASE_INSTALL:-0}"

RUNTIME_URL="${K230_NNCASE_RUNTIME_URL:-https://github.com/kendryte/nncase/releases/download/v${NNCASE_VERSION}/nncase_k230_v${NNCASE_VERSION}_runtime_linux.tgz}"
RUNTIME_SHA256="${K230_NNCASE_RUNTIME_SHA256:-28680932ac879d8591fbaaaab7b8c1ee2d305c2a82471fb2f38c449316cfb91f}"
GSL_URL="${K230_GSL_LITE_URL:-https://raw.githubusercontent.com/gsl-lite/gsl-lite/v0.37.0/include/gsl/gsl-lite.hpp}"
GSL_SHA256="${K230_GSL_LITE_SHA256:-656894aefe55fd316a6b2e777fc7e103898e41ad501abeeb7c4a5c3af82ed7be}"

check_sha256() {
  local expected="$1"
  local path="$2"
  if [[ -n "$expected" ]]; then
    printf '%s  %s\n' "$expected" "$path" | shasum -a 256 -c -
  fi
}

have_runtime() {
  [[ -f "$OUT/include/nncase/version.h" ]] &&
  grep -q "#define NNCASE_VERSION \"$NNCASE_VERSION\"" "$OUT/include/nncase/version.h" &&
  [[ -f "$OUT/include/gsl/gsl-lite.hpp" ]] &&
  [[ -f "$OUT/lib/libNncase.Runtime.Native.a" ]] &&
  [[ -f "$OUT/lib/libnncase.rt_modules.k230.a" ]] &&
  [[ -f "$OUT/lib/libfunctional_k230.a" ]]
}

if [[ "$FORCE" != "1" ]] && have_runtime; then
  echo "K230 nncase runtime $NNCASE_VERSION already installed at $OUT"
  exit 0
fi

TMP_ROOT="$(mktemp -d)"
trap 'rm -rf "$TMP_ROOT"' EXIT

mkdir -p "$OUT"
curl -L --fail --retry 3 -o "$TMP_ROOT/nncase_runtime.tgz" "$RUNTIME_URL"
check_sha256 "$RUNTIME_SHA256" "$TMP_ROOT/nncase_runtime.tgz"

mkdir -p "$TMP_ROOT/extract"
tar -xzf "$TMP_ROOT/nncase_runtime.tgz" -C "$TMP_ROOT/extract"
RUNTIME_DIR="$(find "$TMP_ROOT/extract" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
if [[ -z "$RUNTIME_DIR" || ! -d "$RUNTIME_DIR/include/nncase" || ! -d "$RUNTIME_DIR/lib" ]]; then
  echo "unexpected nncase runtime archive layout" >&2
  exit 1
fi

rm -rf "$OUT/include/nncase" "$OUT/lib"
mkdir -p "$OUT/include" "$OUT/include/gsl"
cp -R "$RUNTIME_DIR/include/nncase" "$OUT/include/"
cp -R "$RUNTIME_DIR/lib" "$OUT/"

curl -L --fail --retry 3 -o "$OUT/include/gsl/gsl-lite.hpp" "$GSL_URL"
check_sha256 "$GSL_SHA256" "$OUT/include/gsl/gsl-lite.hpp"

have_runtime
echo "Installed K230 nncase runtime $NNCASE_VERSION at $OUT"
