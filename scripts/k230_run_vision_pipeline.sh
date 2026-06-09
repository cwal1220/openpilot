#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$ROOT"

DEFAULT_BLOCK="ui,soundd,manage_athenad,pandad,clocksd,dmonitoringmodeld,logcatd,proclogd,sensord,ubloxd,locationd,calibrationd,controlsd,dmonitoringd,paramsd,plannerd,radard"
export BLOCK="${BLOCK:-$DEFAULT_BLOCK}"
export NOBOARD="${NOBOARD:-1}"
export PASSIVE="${PASSIVE:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

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
}

trap cleanup INT TERM EXIT

set_openpilot_view 0
"$ROOT/scripts/k230_run_openpilot.sh" ./selfdrive/manager/manager.py &
manager_pid=$!

sleep "${K230_MANAGER_START_DELAY:-5}"
set_openpilot_view 1

wait "$manager_pid"
