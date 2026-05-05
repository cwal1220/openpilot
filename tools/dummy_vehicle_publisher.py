#!/usr/bin/env python3
import argparse
import os
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
os.environ.setdefault("PYTHONPATH", str(ROOT))
sys.path.insert(0, str(ROOT))

import cereal.messaging as messaging  # noqa: E402


CAL_UNCALIBRATED = 0
CAL_CALIBRATED = 1


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Publish dummy vehicle state for Orange Pi bench testing.")
  parser.add_argument("--speed-kph", type=float, default=60.0)
  parser.add_argument("--rate-hz", type=float, default=50.0)
  parser.add_argument("--log-every", type=int, default=100)
  parser.add_argument("--no-calibration-alerts", action="store_true")
  return parser.parse_args()


def fill_controls_state(msg, speed_kph: float, cal_status: int, cal_perc: int, calibration_alerts: bool) -> None:
  msg.valid = True
  cs = msg.controlsState
  cs.enabled = False
  cs.active = False
  cs.engageable = True
  cs.state = "disabled"
  cs.alertStatus = "normal"
  cs.alertSound = "none"
  cs.vCruise = speed_kph
  cs.lateralControlMethod = 0
  cs.steerRatio = 13.0
  cs.longControlState = "off"

  if calibration_alerts and cal_status == CAL_UNCALIBRATED:
    cs.alertSize = "mid"
    cs.alertText1 = f"Calibration in Progress: {cal_perc}%"
    cs.alertText2 = "Drive Above 24 km/h"
    cs.alertType = "calibrationIncomplete/permanent"
  elif calibration_alerts and cal_status != CAL_CALIBRATED:
    cs.alertSize = "mid"
    cs.alertText1 = "Calibration Invalid"
    cs.alertText2 = "Remount Device & Recalibrate"
    cs.alertType = "calibrationInvalid/permanent"
  else:
    cs.alertSize = "none"
    cs.alertText1 = ""
    cs.alertText2 = ""
    cs.alertType = ""


def fill_car_state(msg, speed_ms: float) -> None:
  msg.valid = True
  cs = msg.carState
  cs.vEgo = speed_ms
  cs.vEgoRaw = speed_ms
  cs.aEgo = 0.0
  cs.standstill = False
  cs.standStill = False
  cs.gearShifter = "drive"
  cs.canValid = True
  cs.cruiseState.available = True
  cs.cruiseState.enabled = False
  cs.cruiseState.standstill = False
  cs.cruiseState.speed = speed_ms
  cs.cruiseGapSet = 3


def fill_car_params(msg) -> None:
  msg.valid = True
  cp = msg.carParams
  cp.carName = "hyundai"
  cp.carFingerprint = "DUMMY HYUNDAI"
  cp.dashcamOnly = False
  cp.openpilotLongitudinalControl = False
  cp.pcmCruise = True
  cp.steerControlType = "torque"
  cp.transmissionType = "automatic"
  cp.mass = 1600.0
  cp.wheelbase = 2.8
  cp.centerToFront = 1.1
  cp.steerRatio = 13.0


def fill_panda_states(msg) -> None:
  msg.valid = True
  ps = msg.pandaStates[0]
  ps.pandaType = "blackPanda"
  ps.ignitionLine = True
  ps.ignitionCan = True
  ps.controlsAllowed = False


def main() -> None:
  args = parse_args()
  speed_ms = args.speed_kph / 3.6
  delay = 1.0 / args.rate_hz
  calibration_alerts = not args.no_calibration_alerts

  pm = messaging.PubMaster(["controlsState", "carState", "carParams", "pandaStates"])
  sm = messaging.SubMaster(["liveCalibration"])

  frame = 0
  print(f"dummy_vehicle_publisher: started speed_kph={args.speed_kph:g}", flush=True)

  while True:
    sm.update(0)
    lc = sm["liveCalibration"]
    cal_status = int(lc.calStatus)
    cal_perc = int(lc.calPerc)

    controls = messaging.new_message("controlsState")
    fill_controls_state(controls, args.speed_kph, cal_status, cal_perc, calibration_alerts)
    pm.send("controlsState", controls)

    car_state = messaging.new_message("carState")
    fill_car_state(car_state, speed_ms)
    pm.send("carState", car_state)

    car_params = messaging.new_message("carParams")
    fill_car_params(car_params)
    pm.send("carParams", car_params)

    panda_states = messaging.new_message("pandaStates", 1)
    fill_panda_states(panda_states)
    pm.send("pandaStates", panda_states)

    frame += 1
    if args.log_every > 0 and frame % args.log_every == 0:
      alert = controls.controlsState.alertText1
      print(
        f"dummy_vehicle_publisher: speed_kph={args.speed_kph:g} "
        f"calStatus={cal_status} calPerc={cal_perc} alert={alert!r}",
        flush=True,
      )

    time.sleep(delay)


if __name__ == "__main__":
  main()
