import os

from selfdrive.hardware import EON, TICI, PC
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

from common.params import Params

WEBCAM = os.getenv("USE_WEBCAM") is not None
K230 = os.getenv("OPENPILOT_TARGET_ARCH") == "riscv64" or os.uname().machine == "riscv64"

EnableLogger = Params().get_bool('OpkrEnableLogger')
EnableUploader = Params().get_bool('OpkrEnableUploader')
EnableShutdownD = Params().get_bool('C2WithCommaPower')
EnableRTShield = Params().get_bool('RTShield')
EnableDriverMonitoring = not K230 and (not PC or WEBCAM)
EnableWebUI = os.getenv("K230_WEBUI", "1" if K230 else "0") != "0"
EnableRadard = os.getenv("K230_LATERAL_ONLY") != "1"

procs = [
  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=not K230, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"], enabled=EnableDriverMonitoring, driverview=True),
  NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"]),
  #NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"]),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"]),
  #NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], enabled=(PC or TICI or EON), persistent=True),
  NativeProcess("previewd", "selfdrive/previewd", ["./previewd"], enabled=K230),
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=not PC, persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=(not PC or WEBCAM)),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"]),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  #PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enabled=EnableDriverMonitoring, driverview=True),
  #PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
  PythonProcess("pandad", "selfdrive.boardd.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard", enabled=EnableRadard),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=TICI, persistent=True),
  PythonProcess("webuid", "selfdrive.webui.webuid", enabled=EnableWebUI, persistent=True),
  #PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, persistent=True),
  #PythonProcess("updated", "selfdrive.updated", enabled=not PC, persistent=True),
  #PythonProcess("uploader", "selfdrive.loggerd.uploader", persistent=True),
  #PythonProcess("statsd", "selfdrive.statsd", persistent=True),
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
if EnableShutdownD:
  procs += [
    PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  ]
if EnableRTShield:
  procs += [
    PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  ]
managed_processes = {p.name: p for p in procs}
