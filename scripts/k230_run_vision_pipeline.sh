#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$ROOT"

export K230_LATERAL_ONLY="${K230_LATERAL_ONLY:-1}"
DEFAULT_BLOCK="ui,soundd,dmonitoringmodeld,dmonitoringd,sensord,clocksd,logcatd,proclogd"
if [[ "${K230_LATERAL_ONLY:-0}" == "1" ]]; then
  DEFAULT_BLOCK="$DEFAULT_BLOCK,radard"
fi
export BLOCK="${BLOCK:-$DEFAULT_BLOCK}"
export PASSIVE="${PASSIVE:-0}"
export NOSENSOR="${NOSENSOR:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

stop_stale_openpilot_processes() {
  python3 - <<'PY'
import os
import signal
import time

needles = (
  "k230_replay_can.py",
  "selfdrive/manager/manager.py",
  "selfdrive.boardd.pandad",
  "selfdrive.thermald.thermald",
  "selfdrive.webui.webuid",
  "selfdrive.locationd.calibrationd",
  "selfdrive.locationd.paramsd",
  "selfdrive.controls.controlsd",
  "selfdrive.controls.plannerd",
  "selfdrive.controls.radard",
  "camerad",
  "_modeld",
  "previewd",
  "locationd",
)

exclude = set()
pid = os.getpid()
while pid > 1:
  exclude.add(pid)
  try:
    with open(f"/proc/{pid}/stat") as f:
      pid = int(f.read().split()[3])
  except Exception:
    break

pids = []
for name in os.listdir("/proc"):
  if not name.isdigit():
    continue
  pid = int(name)
  if pid in exclude:
    continue
  try:
    with open(f"/proc/{pid}/comm") as f:
      comm = f.read().strip()
    with open(f"/proc/{pid}/cmdline", "rb") as f:
      cmd = f.read().replace(b"\0", b" ").decode("utf8", "ignore")
  except Exception:
    continue
  text = f"{comm} {cmd}"
  if any(needle in text for needle in needles):
    pids.append(pid)

for pid in pids:
  try:
    os.kill(pid, signal.SIGTERM)
  except ProcessLookupError:
    pass

time.sleep(2)

for pid in pids:
  try:
    os.kill(pid, signal.SIGKILL)
  except ProcessLookupError:
    pass
PY
}

set_openpilot_view() {
  local enabled="$1"
  "$ROOT/scripts/k230_run_openpilot.sh" python3 - "$enabled" <<'PY'
import sys

from common.params import Params
from selfdrive.version import terms_version, training_version

params = Params()
params.put("HasAcceptedTerms", terms_version)
params.put("CompletedTrainingVersion", training_version)
params.put_bool("IsDriverViewEnabled", False)
params.put_bool("IsOpenpilotViewEnabled", sys.argv[1] == "1")
PY
}

terminate_tree() {
  local pid="$1"
  local child

  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    terminate_tree "$child"
  done

  kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  trap - INT TERM EXIT
  set_openpilot_view 0 || true
  if [[ -n "${manager_pid:-}" ]]; then
    terminate_tree "$manager_pid"
    wait "$manager_pid" 2>/dev/null || true
  fi
  stop_stale_openpilot_processes || true
}

trap cleanup INT TERM EXIT

stop_stale_openpilot_processes || true
set_openpilot_view 0
"$ROOT/scripts/k230_run_openpilot.sh" ./selfdrive/manager/manager.py &
manager_pid=$!

sleep "${K230_MANAGER_START_DELAY:-5}"
set_openpilot_view 1

wait "$manager_pid"
