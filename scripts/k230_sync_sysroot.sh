#!/usr/bin/env bash
set -euo pipefail

BOARD="${K230_BOARD:-root@192.168.219.106}"
SYSROOT="${K230_SYSROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/k230_sysroot}"
PYTHON_VERSION="${K230_PYTHON_VERSION:-3.12}"
SSH_OPTS=(-o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no)
SSH_CMD=(ssh "${SSH_OPTS[@]}")
RSYNC=(rsync)
if [[ -n "${K230_PASSWORD:-}" ]]; then
  RSYNC=(sshpass -p "$K230_PASSWORD" rsync)
fi

mkdir -p "$SYSROOT/usr"

"${RSYNC[@]}" -a --delete -e "${SSH_CMD[*]}" "$BOARD:/usr/include/" "$SYSROOT/usr/include/"
"${RSYNC[@]}" -a --delete -e "${SSH_CMD[*]}" "$BOARD:/usr/lib/riscv64-linux-gnu/" "$SYSROOT/usr/lib/riscv64-linux-gnu/"
"${RSYNC[@]}" -a --delete -e "${SSH_CMD[*]}" "$BOARD:/usr/lib/python$PYTHON_VERSION/config-$PYTHON_VERSION-riscv64-linux-gnu/" "$SYSROOT/usr/lib/python$PYTHON_VERSION/config-$PYTHON_VERSION-riscv64-linux-gnu/"
"${RSYNC[@]}" -a --delete -e "${SSH_CMD[*]}" "$BOARD:/usr/share/pkgconfig/" "$SYSROOT/usr/share/pkgconfig/"

rm -rf "$SYSROOT/lib"
ln -s usr/lib "$SYSROOT/lib"
ln -sfn riscv64-linux-gnu/ld-linux-riscv64-lp64d.so.1 "$SYSROOT/usr/lib/ld-linux-riscv64-lp64d.so.1"

test -d "$SYSROOT/usr/include/python$PYTHON_VERSION"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libpython$PYTHON_VERSION.a"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libpython$PYTHON_VERSION.so"
test -d "$SYSROOT/usr/include/riscv64-linux-gnu/qt5"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libQt5Core.so"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libQt5Location.so"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libavcodec.so"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libyuv.so"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libOpenCL.so"
test -e "$SYSROOT/usr/include/systemd/sd-journal.h"
test -e "$SYSROOT/usr/lib/riscv64-linux-gnu/libsystemd.so"
