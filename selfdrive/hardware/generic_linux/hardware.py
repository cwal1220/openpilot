import os
import re
import socket
import subprocess
from pathlib import Path
from typing import Optional

from cereal import log
from selfdrive.hardware.base import ThermalConfig
from selfdrive.hardware.pc.hardware import Pc

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength


class GenericLinux(Pc):
  @staticmethod
  def _run(cmd) -> Optional[str]:
    try:
      return subprocess.check_output(cmd, encoding='utf8').strip()
    except Exception:
      return None

  @staticmethod
  def _default_iface() -> Optional[str]:
    out = GenericLinux._run(["ip", "route", "show", "default"])
    if not out:
      return None

    match = re.search(r"\bdev\s+(\S+)", out)
    if match:
      return match.group(1)
    return None

  @staticmethod
  def _iface_state(iface: str) -> str:
    try:
      return Path(f"/sys/class/net/{iface}/operstate").read_text().strip()
    except Exception:
      return "down"

  @staticmethod
  def _iface_is_wireless(iface: str) -> bool:
    return Path(f"/sys/class/net/{iface}/wireless").exists()

  @staticmethod
  def _map_strength(dbm: int):
    if dbm >= -50:
      return NetworkStrength.great
    if dbm >= -60:
      return NetworkStrength.good
    if dbm >= -70:
      return NetworkStrength.moderate
    return NetworkStrength.poor

  def get_os_version(self):
    try:
      data = Path("/etc/os-release").read_text().splitlines()
      for line in data:
        if line.startswith("PRETTY_NAME="):
          return line.split("=", 1)[1].strip().strip('"')
    except Exception:
      pass
    return "Generic Linux"

  def get_device_type(self):
    model = self._device_tree_model()
    if "Orange Pi" in model:
      return "orangepi"
    return "generic_linux"

  @staticmethod
  def _device_tree_model() -> str:
    try:
      return Path("/proc/device-tree/model").read_bytes().decode("utf8", "ignore").strip("\x00\n ")
    except Exception:
      return ""

  def reboot(self, reason=None):
    subprocess.call(["sudo", "reboot"])

  def uninstall(self):
    print("uninstall")

  def shutdown(self):
    subprocess.call(["sudo", "poweroff"])

  def get_serial(self):
    try:
      serial = Path("/proc/device-tree/serial-number").read_bytes().decode("utf8", "ignore").strip("\x00\n ")
      if serial:
        return serial
    except Exception:
      pass

    try:
      for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("Serial"):
          return line.split(":", 1)[1].strip()
    except Exception:
      pass

    return self.get_device_type()

  def get_network_info(self):
    iface = self._default_iface()
    if iface is None:
      return None

    return {
      "state": self._iface_state(iface).upper(),
      "technology": "wifi" if self._iface_is_wireless(iface) else "ethernet",
      "operator": iface,
      "channel": 0,
      "extra": iface,
    }

  def get_network_type(self):
    iface = self._default_iface()
    if iface is None:
      return NetworkType.none

    if self._iface_is_wireless(iface):
      return NetworkType.wifi
    return NetworkType.ethernet

  def get_sim_info(self):
    return {
      'sim_id': '',
      'mcc_mnc': None,
      'network_type': ["Unknown"],
      'sim_state': ["ABSENT"],
      'data_connected': False
    }

  def get_network_strength(self, network_type):
    iface = self._default_iface()
    if iface is None or network_type == NetworkType.none:
      return NetworkStrength.unknown, "---", "--"

    if network_type == NetworkType.ethernet:
      return NetworkStrength.great, iface, "--"

    connect_name = iface
    out = self._run(["iw", "dev", iface, "link"])
    if out is not None:
      for line in out.splitlines():
        line = line.strip()
        if line.startswith("SSID:"):
          connect_name = line.split(":", 1)[1].strip()
        elif line.startswith("signal:"):
          try:
            dbm = int(float(line.split(":", 1)[1].strip().split(" ")[0]))
            return self._map_strength(dbm), connect_name, "--"
          except Exception:
            pass

    return NetworkStrength.unknown, connect_name, "--"

  def get_networks(self):
    interfaces = []
    for iface in sorted(os.listdir("/sys/class/net")):
      if iface == "lo":
        continue
      interfaces.append({
        "name": iface,
        "state": self._iface_state(iface).upper(),
        "type": "wifi" if self._iface_is_wireless(iface) else "ethernet",
      })
    return {"interfaces": interfaces}

  def get_ip_address(self):
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
      return "N/A"

  def get_thermal_config(self):
    zones_by_type = {}
    try:
      for name in sorted(os.listdir("/sys/devices/virtual/thermal")):
        if not name.startswith("thermal_zone"):
          continue
        zone = int(name.removeprefix("thermal_zone"))
        zone_type = Path(f"/sys/devices/virtual/thermal/{name}/type").read_text().strip()
        zones_by_type[zone_type] = zone
    except Exception:
      pass

    preferred_cpu_zones = ("cpu-thermal", "soc-thermal", "bigcore0-thermal", "bigcore1-thermal", "littlecore-thermal")
    cpu_zones = tuple(zones_by_type[name] for name in preferred_cpu_zones if name in zones_by_type)
    if len(cpu_zones) == 0:
      cpu_zones = tuple(zone for name, zone in zones_by_type.items() if "cpu" in name or "core" in name or "soc" in name)
    if len(cpu_zones) == 0:
      cpu_zones = (0,)

    gpu_zone = zones_by_type.get("gpu-thermal")

    return ThermalConfig(
      cpu=(cpu_zones, 1000),
      gpu=((gpu_zone,), 1000) if gpu_zone is not None else ((None,), 1),
      mem=(None, 1),
      bat=(None, 1),
      ambient=(None, 1),
      pmic=((None,), 1),
    )

  def set_screen_brightness(self, percentage):
    pass

  def get_screen_brightness(self):
    return 0

  def set_power_save(self, powersave_enabled):
    pass

  def get_gpu_usage_percent(self):
    return 0

  def get_modem_temperatures(self):
    return []

  def get_nvme_temperatures(self):
    return []

  def initialize_hardware(self):
    pass
