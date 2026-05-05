import os
from pathlib import Path
from typing import cast

from selfdrive.hardware.base import HardwareBase

EON = os.path.isfile('/EON')
TICI = os.path.isfile('/TICI')

def _is_generic_linux() -> bool:
  if os.getenv("OPENPILOT_BUILD_PLATFORM") == "linux_generic":
    return True
  try:
    model = Path('/proc/device-tree/model').read_bytes()
    return b'Orange Pi' in model
  except Exception:
    return False

GENERIC_LINUX = _is_generic_linux()
PC = not (EON or TICI or GENERIC_LINUX)


if EON:
  from selfdrive.hardware.eon.hardware import Android
  HARDWARE = cast(HardwareBase, Android())
elif TICI:
  from selfdrive.hardware.tici.hardware import Tici
  HARDWARE = cast(HardwareBase, Tici())
elif GENERIC_LINUX:
  from selfdrive.hardware.generic_linux.hardware import GenericLinux
  HARDWARE = cast(HardwareBase, GenericLinux())
else:
  from selfdrive.hardware.pc.hardware import Pc
  HARDWARE = cast(HardwareBase, Pc())
