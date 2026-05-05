import os

from selfdrive.hardware import EON, TICI, PC, GENERIC_LINUX
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

from common.params import Params

WEBCAM = os.getenv("USE_WEBCAM") is not None
NO_SENSOR = os.getenv("NOSENSOR") is not None
DISABLE_DRIVER_MONITORING = GENERIC_LINUX or (os.getenv("DISABLE_DRIVER_MONITORING") is not None)

EnableLogger = Params().get_bool('OpkrEnableLogger')
EnableUploader = Params().get_bool('OpkrEnableUploader')
EnableOSM = Params().get_bool('OSMEnable') or Params().get_bool('OSMSpeedLimitEnable') or Params().get("CurvDecelOption", encoding="utf8") == "1" or Params().get("CurvDecelOption", encoding="utf8") == "3"
EnableMapbox = Params().get_bool('MapboxEnabled')
EnableShutdownD = Params().get_bool('C2WithCommaPower')
EnableRTShield = Params().get_bool('RTShield')
EnableExternalNavi = Params().get("OPKRNaviSelect", encoding="utf8") == "4" or Params().get("OPKRNaviSelect", encoding="utf8") == "5"

procs = [
  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=True, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"],
                enabled=((not PC or WEBCAM) and not DISABLE_DRIVER_MONITORING), driverview=True),
  NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"], enabled=(not PC and not GENERIC_LINUX)),
  #NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"]),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"]),
  #NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], enabled=(PC or TICI or EON), persistent=True),
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=(not PC and not GENERIC_LINUX and not NO_SENSOR), persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=((not PC or WEBCAM) and not GENERIC_LINUX and not NO_SENSOR)),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"]),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  #PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd",
                enabled=((not PC or WEBCAM) and not DISABLE_DRIVER_MONITORING), driverview=True),
  #PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
  PythonProcess("pandad", "selfdrive.boardd.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=TICI, persistent=True),
  #PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, persistent=True),
  #PythonProcess("updated", "selfdrive.updated", enabled=not PC, persistent=True),
  #PythonProcess("uploader", "selfdrive.loggerd.uploader", persistent=True),
  #PythonProcess("statsd", "selfdrive.statsd", persistent=True),
  #PythonProcess("mapd", "selfdrive.mapd.mapd", enabled=not PC, persistent=True),
  # EON only
  #PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  #PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "selfdrive.hardware.eon.androidd", enabled=EON, persistent=True),
  #PythonProcess("gpxd", "selfdrive.dragonpilot.gpxd"),
  #PythonProcess("otisserv", "selfdrive.dragonpilot.otisserv", persistent=True),

  # Experimental
  #PythonProcess("rawgpsd", "selfdrive.sensord.rawgps.rawgpsd", enabled=os.path.isfile("/persist/comma/use-quectel-rawgps")),
]

if EnableLogger:
  procs += [
    NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"]),
    PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
    PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, persistent=True),
  ]
if EnableUploader:
  procs += [
    PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
    PythonProcess("uploader", "selfdrive.loggerd.uploader", persistent=True),
  ]
if EnableOSM:
  procs += [
    PythonProcess("mapd", "selfdrive.mapd.mapd", enabled=not PC, persistent=True),
  ]
if EnableMapbox:
  procs += [
    PythonProcess("gpxd", "selfdrive.dragonpilot.gpxd"),
    PythonProcess("otisserv", "selfdrive.dragonpilot.otisserv", persistent=True),
  ]
if EnableShutdownD:
  procs += [
    PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  ]
if EnableRTShield:
  procs += [
    PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  ]
if EnableExternalNavi:
  procs += [
    PythonProcess("navid", "selfdrive.navi.navi_external", persistent=True),
  ]

managed_processes = {p.name: p for p in procs}
