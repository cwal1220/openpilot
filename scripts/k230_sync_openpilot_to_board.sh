#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOARD="${K230_BOARD_SSH:-root@192.168.219.106}"
DEST="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
PASSWORD="${K230_BOARD_PASSWORD:-root}"
export SSHPASS="$PASSWORD"

SSH=(sshpass -e ssh -o ConnectTimeout=10 -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null)
RSYNC_RSH="sshpass -e ssh -o ConnectTimeout=10 -o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

rsync_args=(
  -az
  --stats
  --progress
  --delete
  --exclude=/.git/
  --exclude=/.sconsign.dblite
  --exclude=/k230_sysroot/
  --exclude=*.a
  --exclude=*.o
  --exclude=/tools/k230/
)

"${SSH[@]}" "$BOARD" "mkdir -p '$DEST'"

rsync "${rsync_args[@]}" -e "$RSYNC_RSH" "$ROOT/" "$BOARD:$DEST/"

"${SSH[@]}" "$BOARD" "chmod +x '$DEST'/scripts/k230_*.sh"
