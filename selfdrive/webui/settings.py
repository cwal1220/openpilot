import ast
import datetime
import json
import os
import re
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional

BASEDIR = os.getenv("BASEDIR", str(Path(__file__).resolve().parents[2]))
KO_TRANSLATION_FILE = Path(BASEDIR) / "selfdrive/ui/translations/main_ko.ts"
K7_HEV_MODEL = "KIA K7 HYBRID (YG)"
LANG_EN = "main_en"
LANG_KO = "main_ko"
SUPPORTED_LANGUAGES = {LANG_EN, LANG_KO}


@dataclass(frozen=True)
class Option:
  value: str
  label: str

  def to_dict(self) -> Dict[str, str]:
    return {"value": self.value, "label": self.label}


@dataclass(frozen=True)
class Setting:
  key: str
  title: str
  panel: str
  type: str = "text"
  description: str = ""
  default: Optional[str] = None
  step: Optional[float] = None
  min: Optional[float] = None
  max: Optional[float] = None
  options: List[Option] = field(default_factory=list)
  readonly: bool = False
  action: Optional[str] = None
  source: Optional[str] = None
  pair_key: Optional[str] = None
  danger: bool = False
  confirm_text: Optional[str] = None
  input_key: Optional[str] = None
  input_label: Optional[str] = None
  input_placeholder: Optional[str] = None

  def to_dict(self, translate: Optional[Callable[[str], str]] = None) -> Dict[str, Any]:
    tr = translate or (lambda text: text)
    return {
      "key": self.key,
      "title": tr(self.title),
      "panel": self.panel,
      "type": self.type,
      "description": tr(self.description),
      "default": self.default,
      "step": self.step,
      "min": self.min,
      "max": self.max,
      "options": [{"value": o.value, "label": tr(o.label)} for o in self.options],
      "readonly": self.readonly,
      "action": self.action,
      "source": self.source,
      "pair_key": self.pair_key,
      "danger": self.danger,
      "confirm_text": self.confirm_text,
      "input_key": self.input_key,
      "input_label": tr(self.input_label or ""),
      "input_placeholder": tr(self.input_placeholder or ""),
    }


def opt(*pairs: tuple[str, str]) -> List[Option]:
  return [Option(str(value), label) for value, label in pairs]


def b(panel: str, key: str, title: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="bool", description=description)


def n(panel: str, key: str, title: str, step: float = 1, min_: Optional[float] = None,
      max_: Optional[float] = None, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="number", step=step, min=min_, max=max_, description=description)


def e(panel: str, key: str, title: str, options: List[Option], description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="enum", options=options, description=description)


def t(panel: str, key: str, title: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="text", description=description)


def csv(panel: str, key: str, title: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="csv", description=description)


def csv_pair(panel: str, key: str, title: str, pair_key: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="csv", description=description, pair_key=pair_key)


def ro(panel: str, key: str, title: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="readonly", readonly=True, description=description)


def action(panel: str, name: str, title: str, description: str = "", danger: bool = False,
           confirm_text: Optional[str] = None, input_key: Optional[str] = None,
           input_label: Optional[str] = None, input_placeholder: Optional[str] = None) -> Setting:
  return Setting(panel=panel, key=name, title=title, type="action", readonly=True, action=name,
                 description=description, danger=danger, confirm_text=confirm_text,
                 input_key=input_key, input_label=input_label, input_placeholder=input_placeholder)


def dyn(panel: str, key: str, title: str, source: str, description: str = "") -> Setting:
  return Setting(panel=panel, key=key, title=title, type="enum", source=source, description=description)


PANEL_ORDER = ["Toggles", "Device", "Network", "Software", "UIMenu", "Driving", "Developer", "Tuning", "Debug"]
K230_HEADLESS_EXCLUDED_PANELS = {"UIMenu"}
K230_HEADLESS_PANEL_OVERRIDES = {
  "OpkrAutoShutdown": "Device",
  "OpkrForceShutdown": "Device",
  "OpkrBatteryChargingControl": "Device",
  "OpkrBatteryChargingMin": "Device",
  "OpkrBatteryChargingMax": "Device",
  "OPKRServer": "Network",
  "OPKRServerAPI": "Network",
  "OpkrMonitoringMode": "Driving",
  "OpkrMonitorEyesThreshold": "Driving",
  "OpkrMonitorNormalEyesThreshold": "Driving",
  "OpkrMonitorBlinkThreshold": "Driving",
  "ShowStopLine": "Tuning",
}
K230_HEADLESS_HIDDEN_KEYS = {
  "RecordFront",
  "IsOpenpilotViewEnabled",
  "OpkrHotspotOnBoot",
  "GsmRoaming",
  "GsmApn",
  "GitPullOnBoot",
  "OpkrUIVolumeBoost",
  "OpkrUIBrightness",
  "OpkrAutoScreenOff",
  "OpkrUIBrightnessOff",
  "DoNotDisturbMode",
  "OpkrEnableGetoffAlert",
  "OpkrDrivingRecord",
  "RecordingCount",
  "RecordingQuality",
  "AnimatedRPM",
  "AnimatedRPMMax",
  "HoldForSetting",
  "RTShield",
}
PANELS = [panel for panel in PANEL_ORDER if panel not in K230_HEADLESS_EXCLUDED_PANELS]

_SETTINGS: List[Setting] = [
  b("Toggles", "OpenpilotEnabledToggle", "Enable openpilot"),
  b("Toggles", "IsLdwEnabled", "Enable Lane Departure Warnings"),
  b("Toggles", "IsRHD", "Enable Right-Hand Drive"),
  b("Toggles", "IsMetric", "Use Metric System"),
  b("Toggles", "RecordFront", "Record and Upload Driver Camera"),
  b("Toggles", "EndToEndToggle", "Enable Lane selector Mode"),
  b("Toggles", "OpkrEnableLogger", "Enable Driving Log Record"),
  b("Toggles", "OpkrEnableUploader", "Enable Sending Log to Server"),
  b("Toggles", "DisableRadar", "openpilot Longitudinal Control"),

  ro("Device", "DongleId", "Dongle ID"),
  ro("Device", "HardwareSerial", "Serial"),
  b("Device", "IsOpenpilotViewEnabled", "Driving Camera"),
  e("Device", "LanguageSetting", "Change Language", opt((LANG_EN, "English"), (LANG_KO, "한국어"))),
  action("Device", "reset_calibration", "Reset Calibration", "Clear CalibrationParams and LiveParameters."),
  action("Device", "refresh", "Refresh", "Pulse OnRoadRefresh so openpilot reloads settings."),
  action("Device", "reboot", "Reboot", "Ask manager to reboot the device.", True, "REBOOT"),
  action("Device", "shutdown", "Power Off", "Ask manager to shut down the device.", True, "SHUTDOWN"),

  ro("Network", "_ip_address", "IP Address"),
  b("Network", "SshEnabled", "Enable SSH"),
  b("Network", "OpkrSSHLegacy", "Use Legacy SSH Key"),
  t("Network", "GithubUsername", "GitHub Username"),
  t("Network", "GithubSshKeys", "GitHub SSH Keys"),
  action("Network", "fetch_github_ssh_keys", "SSH Keys", "Fetch public SSH keys from GitHub.",
         input_key="username", input_label="GitHub Username", input_placeholder="Enter your GitHub username"),
  action("Network", "remove_github_ssh_keys", "Remove SSH Keys", "Remove GitHub SSH keys and disable legacy SSH key mode."),
  b("Network", "OpkrHotspotOnBoot", "HotSpot on Boot"),

  ro("Software", "GitRemote", "Git Remote"),
  ro("Software", "GitBranch", "Git Branch"),
  ro("Software", "GitCommit", "Git Commit"),
  ro("Software", "GitCommitRemote", "Git Commit Remote"),
  ro("Software", "LastUpdateTime", "Last Update Check"),
  action("Software", "check_update", "Check for Updates", "Fetch origin and compare local and remote commits."),
  action("Software", "git_pull", "Git Pull", "Run git pull for the current branch.", True, "PULL"),
  b("Software", "GitPullOnBoot", "Git Pull On Boot"),
  dyn("Software", "CarModel", "Select Your Car", "car_model"),
  dyn("Software", "OPKRTimeZone", "Select Your TimeZone", "timezone"),

  n("UIMenu", "OpkrAutoShutdown", "Auto Shutdown", 1),
  n("UIMenu", "OpkrForceShutdown", "Force Shutdown", 1),
  n("UIMenu", "OpkrUIVolumeBoost", "EON Volume Control (%)", 1, 0, 100),
  n("UIMenu", "OpkrUIBrightness", "EON Brightness Control (%)", 1, 0, 100),
  n("UIMenu", "OpkrAutoScreenOff", "EON SCR Off Timer", 1),
  n("UIMenu", "OpkrUIBrightnessOff", "Brightness at SCR Off (%)", 1, 0, 100),
  e("UIMenu", "DoNotDisturbMode", "DoNotDisturb Mode", opt(("0", "Off"), ("1", "Screen"), ("2", "Sound"), ("3", "Screen + Sound"))),
  e("UIMenu", "OpkrEnableGetoffAlert", "EON Detach Alert Sound", opt(("0", "No Alert"), ("1", "KOR"), ("2", "ENG"))),
  b("UIMenu", "OpkrBatteryChargingControl", "Enable Battery Charging Control"),
  n("UIMenu", "OpkrBatteryChargingMin", "Battery Min Charge (%)", 1, 0, 100),
  n("UIMenu", "OpkrBatteryChargingMax", "Battery Max Charge (%)", 1, 0, 100),
  b("UIMenu", "OpkrDrivingRecord", "Use Auto Screen Record"),
  n("UIMenu", "RecordingCount", "Number of Recorded Files", 1, 1),
  e("UIMenu", "RecordingQuality", "Recording Quality", opt(("0", "Low"), ("1", "Mid"), ("2", "High"), ("3", "FHD"), ("4", "UHD"))),
  b("UIMenu", "OpkrMonitoringMode", "Driver Monitoring Mode"),
  n("UIMenu", "OpkrMonitorEyesThreshold", "E2E EYE Threshold", 1, 0, 100),
  n("UIMenu", "OpkrMonitorNormalEyesThreshold", "Normal EYE Threshold", 1, 0, 100),
  n("UIMenu", "OpkrMonitorBlinkThreshold", "Blink Threshold", 1, 0, 100),
  e("UIMenu", "OPKRServer", "API Server", opt(("0", "OPKR"), ("1", "Comma"), ("2", "User"))),
  t("UIMenu", "OPKRServerAPI", "User API"),
  b("UIMenu", "AnimatedRPM", "RPM Animated"),
  n("UIMenu", "AnimatedRPMMax", "AnimatedRPM Max", 100, 0),
  b("UIMenu", "ShowStopLine", "Use Stop Line Model"),
  b("UIMenu", "HoldForSetting", "Hold Button for Setting Menu"),
  b("UIMenu", "RTShield", "Enable RTShield Process"),

  b("Driving", "OpkrAutoResume", "Use Auto Resume at Stop"),
  n("Driving", "RESCountatStandstill", "RES Count at Standstill", 1, 0),
  b("Driving", "CruiseGapAdjust", "Change Cruise Gap at Stop"),
  b("Driving", "CruiseGapBySpdOn", "Cruise Gap Change by Speed"),
  csv("Driving", "CruiseGapBySpdSpd", "Cruise Gap by Speed: Speeds"),
  csv("Driving", "CruiseGapBySpdGap", "Cruise Gap by Speed: Gaps"),
  b("Driving", "StandstillResumeAlt", "Standstill Resume Alternative"),
  b("Driving", "DepartChimeAtResume", "Depart Chime at Resume"),
  b("Driving", "OpkrVariableCruise", "Use Cruise Button Spamming"),
  n("Driving", "VarCruiseSpeedFactor", "Button Spamming Level", 1, 0),
  b("Driving", "CruiseSetwithRoadLimitSpeedEnabled", "CruiseSet with RoadLimitSpeed"),
  n("Driving", "CruiseSetwithRoadLimitSpeedOffset", "CruiseSet RoadLimitSpd Ofs", 1),
  e("Driving", "CruiseStatemodeSelInit", "Cruise Start Mode", opt(("0", "OP mode"), ("1", "Dist + Curve"), ("2", "Dist only"), ("3", "Curve only"), ("4", "One-way"), ("5", "SafetyCam only"))),
  n("Driving", "OpkrLaneChangeSpeed", "LaneChange On/Off/Spd", 1),
  n("Driving", "OpkrAutoLaneChangeDelay", "LaneChange Delay", 0.1, 0),
  b("Driving", "LCTimingFactorEnable", "LaneChange Time Enable"),
  e("Driving", "LCTimingFactorUD", "LaneChange Time Mode", opt(("0", "Off"), ("1", "User Defined"))),
  n("Driving", "LCTimingFactor30", "LaneChange Time 30", 1),
  n("Driving", "LCTimingFactor60", "LaneChange Time 60", 1),
  n("Driving", "LCTimingFactor80", "LaneChange Time 80", 1),
  n("Driving", "LCTimingFactor110", "LaneChange Time 110", 1),
  n("Driving", "LeftCurvOffsetAdj", "LeftCurv Offset", 1),
  n("Driving", "RightCurvOffsetAdj", "RightCurv Offset", 1),
  b("Driving", "OpkrBlindSpotDetect", "Show BSM Status"),
  e("Driving", "OpkrSteerMethod", "Steer control Method", opt(("0", "Normal"), ("1", "Smooth"))),
  n("Driving", "OpkrMaxAngleLimit", "Max Steering Angle", 10, 80, 360),
  n("Driving", "OpkrMaxSteeringAngle", "Driver to Steer Angle", 5, 10, 180),
  n("Driving", "OpkrMaxDriverAngleWait", "Driver to Steer", 0.001, 0, 1),
  n("Driving", "OpkrMaxSteerAngleWait", "Steer Angle", 0.001, 0, 1),
  n("Driving", "OpkrDriverAngleWait", "Normal driver to Steer", 0.001, 0, 1),
  n("Driving", "OpkrSteerAngleCorrection", "Str Angle Adjust", 1),
  b("Driving", "OpkrTurnSteeringDisable", "Stop Steer Assist on Turn Signals"),
  b("Driving", "CruiseOverMaxSpeed", "Reset MaxSpeed Over CurrentSpeed"),
  b("Driving", "OSMEnable", "Enable OSM"),
  b("Driving", "OSMSpeedLimitEnable", "Enable OSM SpeedLimit"),
  b("Driving", "StockNaviSpeedEnabled", "Use Stock SafetyCAM Speed"),
  e("Driving", "OpkrSpeedLimitOffsetOption", "SpeedLimit Offset Mode", opt(("0", "Default"), ("1", "Custom"))),
  n("Driving", "OpkrSpeedLimitOffset", "SpeedLimit Offset", 1),
  csv_pair("Driving", "OSMCustomSpeedLimitC", "CustomSpeedLimit: Speed Limit", "OSMCustomSpeedLimitT"),
  csv_pair("Driving", "OSMCustomSpeedLimitT", "CustomSpeedLimit: Target Speed", "OSMCustomSpeedLimitC"),
  e("Driving", "OpkrSpeedLimitSignType", "SafetyCam SignType", opt(("0", "Circle"), ("1", "Rectangle"))),
  n("Driving", "SafetyCamDecelDistGain", "SafetyCamDist Adj (%)", 1),
  e("Driving", "CurvDecelOption", "Curv Decel Option", opt(("0", "None"), ("1", "Vision + OSM"), ("2", "Vision Only"), ("3", "OSM Only"))),
  csv_pair("Driving", "VCurvSpeedC", "VisionCurvDecel: CV", "VCurvSpeedT"),
  csv_pair("Driving", "VCurvSpeedT", "VisionCurvDecel: TargetSpeed", "VCurvSpeedC"),
  csv_pair("Driving", "VCurvSpeedCMPH", "VisionCurvDecel MPH: CV", "VCurvSpeedTMPH"),
  csv_pair("Driving", "VCurvSpeedTMPH", "VisionCurvDecel MPH: TargetSpeed", "VCurvSpeedCMPH"),
  csv_pair("Driving", "OCurvSpeedC", "OSMCurvDecel: TSL", "OCurvSpeedT"),
  csv_pair("Driving", "OCurvSpeedT", "OSMCurvDecel: TargetSpeed", "OCurvSpeedC"),
  b("Driving", "OPKRSpeedBump", "SpeedBump Deceleration"),
  b("Driving", "OPKREarlyStop", "Early Slowdown with Gap"),
  b("Driving", "AutoEnable", "Use Auto Engagement"),
  n("Driving", "AutoEnableSpeed", "Auto Engage Spd (kph)", 1, 0),
  b("Driving", "CruiseAutoRes", "Use Auto RES while Driving"),
  e("Driving", "AutoResOption", "AutoRES Option", opt(("0", "Default"), ("1", "Cruise temporary"), ("2", "Set speed"), ("3", "Lead aware"))),
  e("Driving", "AutoResCondition", "AutoRES Condition", opt(("0", "Brake released"), ("1", "Gas pressed"), ("2", "Both"))),
  n("Driving", "AutoResLimitTime", "AutoRES Allow (sec)", 1, 0),
  n("Driving", "AutoRESDelay", "AutoRES Delay (sec)", 1, 0),
  n("Driving", "LaneWidth", "Set LaneWidth", 1),
  csv_pair("Driving", "SpdLaneWidthSpd", "Speed LaneWidth: Speeds", "SpdLaneWidthSet"),
  csv_pair("Driving", "SpdLaneWidthSet", "Speed LaneWidth: LaneWidths", "SpdLaneWidthSpd"),
  b("Driving", "RoutineDriveOn", "Routine Drive by RoadName"),
  t("Driving", "RoutineDriveOption", "Routine Drive Option"),
  b("Driving", "CloseToRoadEdge", "Driving Close to RoadEdge"),
  n("Driving", "LeftEdgeOffset", "Left Edge Offset", 1),
  n("Driving", "RightEdgeOffset", "Right Edge Offset", 1),
  b("Driving", "AvoidLKASFaultEnabled", "To Avoid LKAS Fault"),
  n("Driving", "AvoidLKASFaultMaxAngle", "Avoid LKAS Fault Max Angle", 1),
  n("Driving", "AvoidLKASFaultMaxFrame", "Avoid LKAS Fault Max Frame", 1),
  b("Driving", "SpeedCameraOffset", "Speed CameraOffset"),

  b("Developer", "DebugUi1", "DEBUG UI 1"),
  b("Developer", "DebugUi2", "DEBUG UI 2"),
  b("Developer", "DebugUi3", "DEBUG UI 3"),
  b("Developer", "OPKRDebug", "OPKR Debug Mode"),
  b("Developer", "ShowError", "Show TMUX Error"),
  b("Developer", "LongLogDisplay", "Show LongControl LOG"),
  b("Developer", "PutPrebuiltOn", "Use Smart Prebuilt"),
  b("Developer", "FingerprintTwoSet", "Use FingerPrint 2.0"),
  b("Developer", "WhitePandaSupport", "Support WhitePanda"),
  b("Developer", "OpkrBattLess", "Set BatteryLess Eon"),
  b("Developer", "ComIssueGone", "Turn Off Communication Issue Alarm"),
  b("Developer", "LdwsCarFix", "Set LDWS Vehicles"),
  b("Developer", "JustDoGearD", "Set DriverGear by Force"),
  b("Developer", "SteerWarningFix", "Ignore of Steering Warning"),
  b("Developer", "IgnoreCANErroronISG", "Ignore Can Error on ISG"),
  b("Developer", "FCA11Message", "Enable FCA11 Message"),
  b("Developer", "UFCModeEnabled", "User-Friendly Control Mode"),
  b("Developer", "StockLKASEnabled", "StockLKAS Enabled at Disengagement"),
  b("Developer", "C2WithCommaPower", "C2 with CommaPower"),
  b("Developer", "JoystickDebugMode", "JoyStick Debug Mode"),
  b("Developer", "NoSmartMDPS", "No Smart MDPS"),
  n("Developer", "UserSpecificFeature", "FeatureNumber", 1, 0),
  e("Developer", "LateralControlMethod", "LatControl", opt(("0", "PID"), ("1", "INDI"), ("2", "LQR"), ("3", "TORQUE"), ("4", "MULTI"))),
  n("Developer", "MaxSteer", "MAX_STEER", 1),
  n("Developer", "MaxRTDelta", "RT_DELTA", 1),
  n("Developer", "MaxRateUp", "MAX_RATE_UP", 1),
  n("Developer", "MaxRateDown", "MAX_RATE_DOWN", 1),

  n("Tuning", "CameraOffsetAdj", "CameraOffset", 1),
  n("Tuning", "PathOffsetAdj", "PathOffset", 1),
  n("Tuning", "SteerActuatorDelayAdj", "SteerActuatorDelay", 1),
  n("Tuning", "TireStiffnessFactorAdj", "TireStiffnessFactor", 1),
  n("Tuning", "SteerThreshold", "SteerThreshold", 1),
  n("Tuning", "SteerLimitTimerAdj", "SteerLimitTimer", 1),
  b("Tuning", "OpkrLiveSteerRatio", "Use Live SteerRatio"),
  n("Tuning", "LiveSteerRatioPercent", "LiveSR Adjust (%)", 1),
  n("Tuning", "SteerRatioAdj", "SteerRatio", 1),
  n("Tuning", "SteerRatioMaxAdj", "SteerRatioMax", 1),
  b("Tuning", "OpkrVariableSteerMax", "SteerMax/Variable SteerMax Toggle"),
  n("Tuning", "SteerMaxBaseAdj", "SteerMax Base", 1),
  n("Tuning", "SteerMaxAdj", "SteerMax Max", 1),
  b("Tuning", "OpkrVariableSteerDelta", "DeltaUpDown/Variable Delta Toggle"),
  n("Tuning", "SteerDeltaUpBaseAdj", "SteerDeltaUp Base", 1),
  n("Tuning", "SteerDeltaUpAdj", "SteerDeltaUp Max", 1),
  n("Tuning", "SteerDeltaDownBaseAdj", "SteerDeltaDown Base", 1),
  n("Tuning", "SteerDeltaDownAdj", "SteerDeltaDown Max", 1),
  b("Tuning", "AvoidLKASFaultBeyond", "To Avoid LKAS Fault with More Steer"),
  n("Tuning", "DesiredCurvatureLimit", "DesiredCurvatureLimit", 1),
  b("Tuning", "OpkrLiveTunePanelEnable", "Use LiveTune and Show UI"),
  n("Tuning", "PidKp", "PID Kp", 1),
  n("Tuning", "PidKi", "PID Ki", 1),
  n("Tuning", "PidKd", "PID Kd", 1),
  n("Tuning", "PidKf", "PID Kf", 1),
  n("Tuning", "InnerLoopGain", "INDI InnerLoopGain", 1),
  n("Tuning", "OuterLoopGain", "INDI OuterLoopGain", 1),
  n("Tuning", "TimeConstant", "INDI TimeConstant", 1),
  n("Tuning", "ActuatorEffectiveness", "INDI ActuatorEffectiveness", 1),
  n("Tuning", "Scale", "LQR Scale", 1),
  n("Tuning", "LqrKi", "LQR Ki", 1),
  n("Tuning", "DcGain", "LQR DcGain", 1),
  n("Tuning", "TorqueMaxLatAccel", "Torque MaxLatAccel", 1),
  n("Tuning", "TorqueKp", "Torque Kp", 1),
  n("Tuning", "TorqueKf", "Torque Kf", 1),
  n("Tuning", "TorqueKi", "Torque Ki", 1),
  n("Tuning", "TorqueFriction", "Torque Friction", 1),
  b("Tuning", "TorqueUseAngle", "Torque UseAngle"),
  n("Tuning", "TorqueAngDeadZone", "Torque AngleDeadZone", 1),
  b("Tuning", "CustomTREnabled", "Custom TR Enable"),
  n("Tuning", "CruiseGap1", "CruiseGap 1", 0.1),
  n("Tuning", "CruiseGap2", "CruiseGap 2", 0.1),
  n("Tuning", "CruiseGap3", "CruiseGap 3", 0.1),
  n("Tuning", "CruiseGap4", "CruiseGap 4", 0.1),
  e("Tuning", "DynamicTRGap", "Use DynamicTR", opt(("0", "Off"), ("1", "Gap 1"), ("2", "Gap 2"), ("3", "Gap 3"), ("4", "Gap 4"))),
  csv_pair("Tuning", "DynamicTRSpd", "DynamicTR Speeds", "DynamicTRSet"),
  csv_pair("Tuning", "DynamicTRSet", "DynamicTR TRs", "DynamicTRSpd"),
  e("Tuning", "RadarLongHelper", "Long Mode", opt(("0", "Vision Only"), ("1", "Radar Only"), ("2", "OPKR"))),
  b("Tuning", "StoppingDistAdj", "Adjust Stopping Distance"),
  n("Tuning", "StoppingDist", "Stopping Distance (m)", 1),
  b("Tuning", "E2ELong", "Enable E2E Long"),
  b("Tuning", "StopAtStopSign", "Stop at Stop Sign"),
  b("Tuning", "UseStockDecelOnSS", "Use Stock Decel on Safety Section"),
  b("Tuning", "RadarDisable", "Disable Radar"),
  e("Tuning", "MultipleLateralUse", "Multi LateralControl", opt(("0", "PID"), ("1", "INDI"), ("2", "LQR"), ("3", "TORQUE"), ("4", "MULTI"))),
  csv_pair("Tuning", "MultipleLateralSpd", "Multiple Lateral Speeds", "MultipleLateralOpS"),
  csv_pair("Tuning", "MultipleLateralOpS", "Multiple Lateral Speed Methods", "MultipleLateralSpd"),
  csv_pair("Tuning", "MultipleLateralAng", "Multiple Lateral Angles", "MultipleLateralOpA"),
  csv_pair("Tuning", "MultipleLateralOpA", "Multiple Lateral Angle Methods", "MultipleLateralAng"),
]

def k230_headless_setting(setting: Setting) -> Optional[Setting]:
  if setting.key in K230_HEADLESS_HIDDEN_KEYS:
    return None
  if setting.key in K230_HEADLESS_PANEL_OVERRIDES:
    return replace(setting, panel=K230_HEADLESS_PANEL_OVERRIDES[setting.key])
  if setting.panel in K230_HEADLESS_EXCLUDED_PANELS:
    return None
  return setting


SETTINGS: List[Setting] = []
for setting in _SETTINGS:
  headless_setting = k230_headless_setting(setting)
  if headless_setting is not None:
    SETTINGS.append(headless_setting)

SETTINGS_BY_KEY = {s.key: s for s in SETTINGS if s.type != "action" and not s.key.startswith("_")}


def load_default_params() -> Dict[str, str]:
  manager_py = Path(BASEDIR) / "selfdrive/manager/manager.py"
  if not manager_py.exists():
    return {}

  text = manager_py.read_text(errors="ignore")
  defaults: Dict[str, str] = {}
  try:
    tree = ast.parse(text)
  except SyntaxError:
    return defaults

  for node in ast.walk(tree):
    value = None
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "default_params":
      value = node.value
    elif isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "default_params" for target in node.targets):
      value = node.value
    if not isinstance(value, ast.List):
      continue
    for item in value.elts:
      if not isinstance(item, ast.Tuple) or len(item.elts) < 2:
        continue
      try:
        key = ast.literal_eval(item.elts[0])
        parsed = ast.literal_eval(item.elts[1])
      except Exception:
        continue
      if isinstance(parsed, bytes):
        parsed = parsed.decode("utf8", errors="replace")
      defaults[str(key)] = str(parsed)
  return defaults


DEFAULT_PARAMS = load_default_params()

LIVE_TUNE_KEYS = {
  "CameraOffsetAdj", "PathOffsetAdj", "PidKp", "PidKi", "PidKd", "PidKf",
  "InnerLoopGain", "OuterLoopGain", "TimeConstant", "ActuatorEffectiveness",
  "Scale", "LqrKi", "DcGain", "TorqueMaxLatAccel", "TorqueKp", "TorqueKf",
  "TorqueKi", "TorqueFriction", "TorqueUseAngle", "TorqueAngDeadZone",
}

@dataclass(frozen=True)
class HelpDoc:
  en: str
  ko: str
  value_en: str = ""
  value_ko: str = ""


def doc(en: str, ko: str, value_en: str = "", value_ko: str = "") -> HelpDoc:
  return HelpDoc(en, ko, value_en, value_ko)


SETTING_DOCS: Dict[str, HelpDoc] = {
  "OpenpilotEnabledToggle": doc(
    "Read by controlsd at startup. When disabled, openpilot remains in dashcam/read-only behavior instead of allowing engagement.",
    "controlsd 시작 시 읽습니다. 꺼두면 openpilot이 조향/가감속 제어로 engage되지 않고 dashcam/read-only 동작에 머뭅니다.",
    "0=disabled, 1=enabled. Restart controlsd/manager for a startup-only change to be guaranteed.",
    "0=비활성, 1=활성. 시작 시 읽는 값이므로 확실한 적용은 controlsd/manager 재시작 후 확인합니다.",
  ),
  "IsLdwEnabled": doc(
    "Read by controlsd for lane-departure warning behavior. It affects warning generation, not the model input or KPU path.",
    "controlsd가 차선 이탈 경고 동작에 사용합니다. 모델 입력이나 KPU 추론 경로를 바꾸는 값은 아닙니다.",
    "0=off, 1=on.",
    "0=꺼짐, 1=켜짐.",
  ),
  "IsRHD": doc(
    "Registered persistent openpilot setting for right-hand-drive layouts. In this K230 bring-up it is retained for compatibility with the original settings page.",
    "우핸들 차량 UI/동작 호환을 위한 openpilot persistent 설정입니다. 현재 K230 포팅에서는 기존 설정 페이지와의 호환 목적으로 유지합니다.",
    "0=left-hand drive, 1=right-hand drive.",
    "0=좌핸들, 1=우핸들.",
  ),
  "IsMetric": doc(
    "Read by controlsd, lateral planner, long MPC, map/navigation helpers, logger display, and UI code to choose kph/km units instead of mph/miles.",
    "controlsd, lateral planner, long MPC, 지도/내비 보조 코드, 로그/표시 코드가 km/h/km 단위와 mph/mile 단위를 선택할 때 읽습니다.",
    "0=imperial, 1=metric.",
    "0=영국식 단위, 1=미터법.",
  ),
  "RecordFront": doc(
    "Read by loggerd and snapshot code to decide whether the driver/front camera stream may be recorded or uploaded.",
    "loggerd와 snapshot 코드가 운전자/전방 실내 카메라 스트림 기록 또는 업로드 허용 여부를 결정할 때 읽습니다.",
    "0=do not record front camera, 1=allow recording when that camera exists.",
    "0=실내/운전자 카메라 기록 안 함, 1=해당 카메라가 있을 때 기록 허용.",
  ),
  "EndToEndToggle": doc(
    "Read by plannerd and lateral_planner. When enabled, lane-line reliance is reduced and the end-to-end model path is preferred.",
    "plannerd와 lateral_planner가 읽습니다. 켜면 차선선 기반 계획보다 end-to-end 모델 경로를 우선 사용합니다.",
    "0=lane-line mode, 1=end-to-end path mode.",
    "0=차선선 기반, 1=end-to-end 경로 기반.",
  ),
  "OpkrEnableLogger": doc(
    "Original OPKR logging toggle. It is registered in params and exposed here, but no active K230 runtime reader was found in the current camerad/modeld/webui path.",
    "기존 OPKR 주행 로그 토글입니다. params에는 등록되어 있고 편집 가능하지만, 현재 K230의 camerad/modeld/webui 실행 경로에서는 활성 reader를 확인하지 못했습니다.",
    "0=off, 1=on for OPKR components that still read it.",
    "0=꺼짐, 1=이 값을 읽는 OPKR 구성요소에서 켜짐.",
  ),
  "OpkrEnableUploader": doc(
    "Original OPKR upload toggle retained for compatibility. The current K230 minimal pipeline does not start the original uploader path by default.",
    "기존 OPKR 업로드 토글을 호환성 때문에 유지합니다. 현재 K230 최소 파이프라인에서는 기존 업로더 경로가 기본으로 동작하지 않습니다.",
    "0=off, 1=on for compatible uploader code.",
    "0=꺼짐, 1=호환 업로더 코드에서 켜짐.",
  ),
  "DisableRadar": doc(
    "Read during car parameter detection. It disables radar use and makes longitudinal planning rely on non-radar sources where supported; openpilot marks this as safety-sensitive.",
    "차량 파라미터 감지 과정에서 읽습니다. 레이더 사용을 끄고 지원되는 경우 비레이더 기반 longitudinal planning을 사용하게 하며, openpilot에서 안전 관련 값으로 취급합니다.",
    "0=radar allowed, 1=radar disabled. Manager may delete it unless DisableRadar_Allow is present.",
    "0=레이더 허용, 1=레이더 비활성. DisableRadar_Allow가 없으면 manager가 값을 지울 수 있습니다.",
  ),

  "DongleId": doc("Read-only device identity Param.", "읽기 전용 장치 식별 Param입니다."),
  "HardwareSerial": doc("Read-only hardware serial Param.", "읽기 전용 하드웨어 시리얼 Param입니다."),
  "IsOpenpilotViewEnabled": doc(
    "Read by the Qt/onroad UI path to switch the driving-camera preview mode. The K230 LCD path is currently not using the full Qt GUI, so this is mostly a compatibility setting.",
    "Qt/onroad UI 경로가 주행 카메라 preview 모드를 바꿀 때 읽습니다. 현재 K230 LCD 경로는 full Qt GUI를 쓰지 않으므로 주로 호환 설정입니다.",
    "0=normal UI view, 1=openpilot camera view.",
    "0=일반 UI view, 1=openpilot 카메라 view.",
  ),
  "LanguageSetting": doc(
    "Read by Qt main.cc, events.py, and this WebUI. Changing it reloads the WebUI schema so menu titles, option labels, and value guides are returned in the selected language.",
    "Qt main.cc, events.py, WebUI가 읽습니다. 변경하면 WebUI schema를 다시 받아 메뉴, 옵션, 값 안내가 선택한 언어로 함께 바뀝니다.",
    "main_en=English, main_ko=Korean. WebUI intentionally supports only these two languages.",
    "main_en=영문, main_ko=한글. WebUI는 의도적으로 이 두 언어만 지원합니다.",
  ),
  "reset_calibration": doc(
    "WebUI action. It deletes CalibrationParams and LiveParameters, then pulses OnRoadRefresh so openpilot rebuilds calibration/live parameter state.",
    "WebUI 액션입니다. CalibrationParams와 LiveParameters를 삭제한 뒤 OnRoadRefresh를 잠깐 켜서 openpilot이 캘리브레이션/라이브 파라미터 상태를 다시 만들게 합니다.",
  ),
  "refresh": doc(
    "WebUI action. It sets OnRoadRefresh true for about three seconds and then clears it, matching the original refresh behavior.",
    "WebUI 액션입니다. OnRoadRefresh를 약 3초 동안 true로 만들었다가 지우며, 기존 refresh 동작과 맞춥니다.",
  ),
  "reboot": doc(
    "WebUI action. It writes DoReboot=true; manager is responsible for observing that Param and rebooting the board.",
    "WebUI 액션입니다. DoReboot=true를 기록하며, manager가 이 Param을 보고 보드를 재부팅합니다.",
  ),
  "shutdown": doc(
    "WebUI action. It writes DoShutdown=true; manager is responsible for observing that Param and powering down.",
    "WebUI 액션입니다. DoShutdown=true를 기록하며, manager가 이 Param을 보고 종료합니다.",
  ),

  "SshEnabled": doc(
    "Read by hardware/SSH management code to decide whether SSH access should be available.",
    "하드웨어/SSH 관리 코드가 SSH 접근 허용 여부를 결정할 때 읽습니다.",
    "0=SSH disabled, 1=SSH enabled.",
    "0=SSH 비활성, 1=SSH 활성.",
  ),
  "OpkrSSHLegacy": doc(
    "Read by thermald and the original SSH key widget. When enabled, the legacy bundled OPKR public key is installed into GithubSshKeys.",
    "thermald와 기존 SSH key 위젯이 읽습니다. 켜면 OPKR legacy 공개키를 GithubSshKeys에 설치합니다.",
    "0=use newer key handling, 1=install legacy key.",
    "0=새 키 처리 사용, 1=legacy 키 설치.",
  ),
  "GithubUsername": doc(
    "Stored together with GithubSshKeys by the SSH-key action. The original SSH widget displays it and WebUI reuses it as the fetch input default.",
    "SSH-key 액션이 GithubSshKeys와 함께 저장합니다. 기존 SSH 위젯이 표시하고 WebUI도 fetch 입력 기본값으로 재사용합니다.",
  ),
  "GithubSshKeys": doc(
    "Read by athenad and SSH setup code as the authorized public key material for remote access.",
    "athenad와 SSH 설정 코드가 원격 접속용 authorized public key 자료로 읽습니다.",
    "One or more OpenSSH public-key lines.",
    "OpenSSH 공개키 한 줄 이상.",
  ),
  "fetch_github_ssh_keys": doc(
    "WebUI action. It downloads https://github.com/<username>.keys, stores GithubUsername/GithubSshKeys, and leaves the key material visible for review.",
    "WebUI 액션입니다. https://github.com/<username>.keys 를 내려받아 GithubUsername/GithubSshKeys에 저장하고, 저장된 키 내용을 확인할 수 있게 합니다.",
  ),
  "remove_github_ssh_keys": doc(
    "WebUI action. It removes GithubUsername and GithubSshKeys, then clears OpkrSSHLegacy.",
    "WebUI 액션입니다. GithubUsername과 GithubSshKeys를 삭제하고 OpkrSSHLegacy를 끕니다.",
  ),
  "OpkrHotspotOnBoot": doc(
    "Original OPKR boot hotspot toggle. It is retained because the Param is registered, but no active K230 reader was found in the current Ubuntu board path.",
    "기존 OPKR 부팅 시 hotspot 토글입니다. Param은 등록되어 있어 유지하지만, 현재 Ubuntu K230 보드 경로에서는 활성 reader를 확인하지 못했습니다.",
  ),
  "GitRemote": doc("Read-only Git remote recorded by manager/WebUI software actions.", "manager/WebUI 소프트웨어 액션이 기록하는 읽기 전용 Git remote 정보입니다."),
  "GitBranch": doc("Read-only Git branch recorded by manager/WebUI software actions.", "manager/WebUI 소프트웨어 액션이 기록하는 읽기 전용 Git branch 정보입니다."),
  "GitCommit": doc("Read-only local Git commit recorded by manager/WebUI software actions.", "manager/WebUI 소프트웨어 액션이 기록하는 읽기 전용 local Git commit입니다."),
  "GitCommitRemote": doc("Read-only remote Git commit recorded after update checks.", "업데이트 확인 후 기록되는 읽기 전용 remote Git commit입니다."),
  "LastUpdateTime": doc("Read-only timestamp updated by WebUI check_update/git_pull actions.", "WebUI check_update/git_pull 액션이 갱신하는 읽기 전용 시간 정보입니다."),
  "check_update": doc(
    "WebUI action. It runs git fetch origin, compares HEAD with origin/<current branch>, writes GitCommitRemote, and updates LastUpdateTime.",
    "WebUI 액션입니다. git fetch origin을 실행한 뒤 HEAD와 origin/<현재 브랜치>를 비교하고 GitCommitRemote 및 LastUpdateTime을 갱신합니다.",
  ),
  "git_pull": doc(
    "WebUI action. It runs git pull --ff-only origin <current branch> and updates GitBranch, GitCommit, GitRemote, and LastUpdateTime when successful.",
    "WebUI 액션입니다. git pull --ff-only origin <현재 브랜치>를 실행하고 성공 시 GitBranch, GitCommit, GitRemote, LastUpdateTime을 갱신합니다.",
  ),
  "GitPullOnBoot": doc(
    "Read by ui.cc during UI initialization in the original Qt path. When enabled there, it checks for updates after boot and can run git pull/reboot.",
    "기존 Qt 경로의 ui.cc가 UI 초기화 중 읽습니다. 해당 경로에서 켜져 있으면 부팅 후 업데이트를 확인하고 git pull/reboot를 수행할 수 있습니다.",
  ),
  "CarModel": doc(
    "Read by car_helpers before fingerprint selection. If set, it force-selects that car profile; the K230 default is KIA K7 HYBRID (YG).",
    "car_helpers가 fingerprint 선택 전에 읽습니다. 값이 있으면 해당 차량 프로필을 강제로 선택하며, K230 기본값은 KIA K7 HYBRID (YG)입니다.",
    "Use the exact model string from selfdrive/car/hyundai/values.py or the generated CarList.",
    "selfdrive/car/hyundai/values.py 또는 생성된 CarList의 차량 문자열을 정확히 사용합니다.",
  ),
  "OPKRTimeZone": doc(
    "Original OPKR timezone Param selected from the TimeZone asset list. It is kept so the former Qt timezone menu remains available in WebUI.",
    "TimeZone asset 목록에서 선택하던 기존 OPKR timezone Param입니다. 기존 Qt timezone 메뉴를 WebUI에서도 유지하기 위해 제공합니다.",
    "IANA timezone name such as UTC or Asia/Seoul.",
    "UTC 또는 Asia/Seoul 같은 IANA timezone 이름.",
  ),

  "OpkrAutoShutdown": doc(
    "Read by thermald. The raw index is converted to an offroad shutdown delay table: 0, 5, 30, 60, 180, 300, 600, 1800, 3600, or 10800 seconds.",
    "thermald가 읽습니다. 원시 index를 offroad 자동 종료 지연 표로 변환합니다: 0, 5, 30, 60, 180, 300, 600, 1800, 3600, 10800초.",
    "0 disables; 2 means 30 seconds in the current table.",
    "0은 비활성, 2는 현재 표 기준 30초입니다.",
  ),
  "OpkrForceShutdown": doc(
    "Read by thermald when the board is discharging and has not seen a started state. The raw index maps to 0, 60, 180, 300, 600, or 1800 seconds.",
    "thermald가 방전 중이고 started 상태를 보지 못했을 때 읽습니다. 원시 index는 0, 60, 180, 300, 600, 1800초로 변환됩니다.",
    "0 disables forced shutdown.",
    "0은 강제 종료 비활성입니다.",
  ),
  "OpkrUIVolumeBoost": doc(
    "Read by soundd. The raw percent is multiplied by 0.01 and applied as the sound volume factor when outside the small deadband.",
    "soundd가 읽습니다. 원시 percent에 0.01을 곱해 작은 deadband 밖에서 사운드 볼륨 계수로 적용합니다.",
    "0 is neutral; positive values raise volume factor.",
    "0은 중립, 양수는 볼륨 계수를 올립니다.",
  ),
  "OpkrUIBrightness": doc(
    "Read by ui.cc and stored in the UI scene as the requested screen brightness override.",
    "ui.cc가 읽어 UI scene의 화면 밝기 override 값으로 저장합니다.",
    "0 lets the existing UI policy decide; 1-100 request a fixed percentage.",
    "0은 기존 UI 정책 사용, 1-100은 고정 밝기 비율 요청.",
  ),
  "OpkrAutoScreenOff": doc(
    "Read by ui.cc as the onroad screen-off timer value. The original Qt control displays this value in minutes.",
    "ui.cc가 onroad 화면 꺼짐 타이머 값으로 읽습니다. 기존 Qt control은 이 값을 분 단위로 표시합니다.",
    "Negative values are legacy special modes; non-negative values are timer minutes in the original UI.",
    "음수는 기존 특수 모드, 0 이상은 기존 UI 기준 타이머 분 값입니다.",
  ),
  "OpkrUIBrightnessOff": doc(
    "Read by ui.cc/window.cc as the brightness level to use while the screen-off mode is active.",
    "ui.cc/window.cc가 screen-off 모드 중 사용할 밝기 값으로 읽습니다.",
    "0-100 percent.",
    "0-100 퍼센트.",
  ),
  "DoNotDisturbMode": doc(
    "Read by ui.cc/window.cc and soundd. It suppresses screen and/or sound notifications depending on the selected mode.",
    "ui.cc/window.cc와 soundd가 읽습니다. 선택 모드에 따라 화면/사운드 알림을 억제합니다.",
    "0=off, 1=screen, 2=sound, 3=screen+sound.",
    "0=꺼짐, 1=화면, 2=소리, 3=화면+소리.",
  ),
  "OpkrBatteryChargingControl": doc(
    "Read by thermald. When enabled, thermald applies the configured minimum/maximum battery charging thresholds.",
    "thermald가 읽습니다. 켜면 설정된 최소/최대 충전 임계값을 적용합니다.",
    "0=off, 1=control charging with min/max values.",
    "0=꺼짐, 1=최소/최대 값으로 충전 제어.",
  ),
  "OpkrBatteryChargingMin": doc(
    "Read by thermald as the lower battery threshold for OPKR charging control.",
    "thermald가 OPKR 충전 제어의 하한 배터리 임계값으로 읽습니다.",
    "0-100 percent. Must be below the max threshold.",
    "0-100 퍼센트. 최대 임계값보다 낮아야 합니다.",
  ),
  "OpkrBatteryChargingMax": doc(
    "Read by thermald as the upper battery threshold for OPKR charging control.",
    "thermald가 OPKR 충전 제어의 상한 배터리 임계값으로 읽습니다.",
    "0-100 percent. Must be above the min threshold.",
    "0-100 퍼센트. 최소 임계값보다 높아야 합니다.",
  ),
  "OPKRServer": doc(
    "Read by athenad and common.api at process startup to choose the remote API/Athena server.",
    "athenad와 common.api가 프로세스 시작 시 읽어 원격 API/Athena 서버를 선택합니다.",
    "0=OPKR, 1=comma, 2=custom OPKRServerAPI. Restart the affected process after changing.",
    "0=OPKR, 1=comma, 2=사용자 OPKRServerAPI입니다. 변경 후 관련 프로세스를 재시작해야 확실히 적용됩니다.",
  ),
  "OPKRServerAPI": doc(
    "Custom host used by athenad and common.api when OPKRServer is set to User.",
    "OPKRServer가 User일 때 athenad와 common.api가 사용하는 사용자 지정 host입니다.",
    "Store only the host[:port]; the code adds wss:// for Athena and http:// for API.",
    "host[:port]만 저장합니다. Athena는 wss://, API는 http://를 코드가 붙입니다.",
  ),
  "OpkrMonitoringMode": doc(
    "Read by driver monitoring and controlsd. It enables the OPKR unsleep/driver-monitoring mode when the monitoring pipeline is running.",
    "driver monitoring과 controlsd가 읽습니다. monitoring 파이프라인이 실행 중일 때 OPKR 졸음/운전자 모니터링 모드를 켭니다.",
    "0=standard monitoring behavior, 1=OPKR monitoring mode.",
    "0=기본 monitoring 동작, 1=OPKR monitoring mode.",
  ),
  "OpkrMonitorEyesThreshold": doc(
    "Read by driver_monitor as the eye-open threshold for OPKR monitoring mode.",
    "driver_monitor가 OPKR monitoring mode의 눈 뜸 임계값으로 읽습니다.",
    "0-100 percent; the code multiplies by 0.01.",
    "0-100 퍼센트이며 코드에서 0.01을 곱합니다.",
  ),
  "OpkrMonitorNormalEyesThreshold": doc(
    "Read by driver_monitor as the normal eye threshold for OPKR monitoring mode.",
    "driver_monitor가 OPKR monitoring mode의 normal eye 임계값으로 읽습니다.",
    "0-100 percent; the code multiplies by 0.01.",
    "0-100 퍼센트이며 코드에서 0.01을 곱합니다.",
  ),
  "OpkrMonitorBlinkThreshold": doc(
    "Read by driver_monitor as the blink threshold for OPKR monitoring mode.",
    "driver_monitor가 OPKR monitoring mode의 blink 임계값으로 읽습니다.",
    "0-100 percent; the code multiplies by 0.01.",
    "0-100 퍼센트이며 코드에서 0.01을 곱합니다.",
  ),

  "LateralControlMethod": doc(
    "Read in Hyundai CarInterface when CarParams are built. It selects which lateral tuning block set_lat_tune installs: PID, INDI, LQR, TORQUE, or ATOM/MULTI.",
    "Hyundai CarInterface가 CarParams를 만들 때 읽습니다. set_lat_tune이 설치할 lateral tuning 블록을 PID, INDI, LQR, TORQUE, ATOM/MULTI 중에서 선택합니다.",
    "0=PID, 1=INDI, 2=LQR, 3=TORQUE, 4=ATOM/MULTI. K7 HEV default is 3.",
    "0=PID, 1=INDI, 2=LQR, 3=TORQUE, 4=ATOM/MULTI. K7 HEV 기본값은 3입니다.",
  ),
  "DebugUi1": doc(
    "Legacy onroad debug toggle. K230 disables drawing those debug strings over the LCD preview; the same controlsState alert/debug data is available in the WebUI Debug tab.",
    "기존 onroad 디버그 토글입니다. K230에서는 LCD preview 위에 이 문자열을 그리지 않도록 했고, 같은 controlsState alert/debug 데이터는 WebUI Debug 탭에서 확인합니다.",
  ),
  "DebugUi2": doc(
    "Legacy lateral/live-parameter debug toggle. For K230, use the WebUI Debug tab so the camera preview overlay stays clean.",
    "기존 lateral/live-parameter 디버그 토글입니다. K230에서는 카메라 preview overlay를 깨끗하게 유지하기 위해 WebUI Debug 탭을 사용합니다.",
  ),
  "DebugUi3": doc(
    "Legacy longitudinal debug toggle. Longitudinal debug strings remain message data and are surfaced through the WebUI Debug endpoint.",
    "기존 longitudinal 디버그 토글입니다. longitudinal 디버그 문자열은 message 데이터로 유지되며 WebUI Debug endpoint로 표시됩니다.",
  ),
  "OPKRDebug": doc(
    "Original OPKR raw debug display toggle. It is kept as a Param editor entry, but K230 preview rendering should not depend on it.",
    "기존 OPKR raw debug 표시 토글입니다. Param 편집 항목으로 유지하지만 K230 preview 렌더링은 이 값에 의존하지 않도록 했습니다.",
  ),
  "LongLogDisplay": doc(
    "Read by longcontrol every 100 control cycles. It changes which longitudinal-control debug string is generated for alertTextMsg/log display.",
    "longcontrol이 100 control cycle마다 읽습니다. alertTextMsg/log 표시용 longitudinal-control 디버그 문자열 생성을 바꿉니다.",
    "0=normal long log, 1=verbose OPKR long log.",
    "0=일반 long log, 1=상세 OPKR long log.",
  ),

  "CameraOffsetAdj": doc(
    "Read by lane_planner. On EON-style paths it becomes camera_offset = -(raw * 0.001) meters and shifts both detected lane lines before path planning; with LiveTune enabled it reloads about once per second.",
    "lane_planner가 읽습니다. EON 계열 경로에서 camera_offset = -(raw * 0.001) m 로 변환되어 검출된 양쪽 차선 y값을 path planning 전에 이동시키며, LiveTune이 켜져 있으면 약 1초마다 다시 읽습니다.",
    "Raw integer millimeters. Positive raw values become negative meters in lane_planner.",
    "원시 정수 mm. 양수 raw 값은 lane_planner 안에서 음수 meter offset이 됩니다.",
  ),
  "PathOffsetAdj": doc(
    "Read by lane_planner.get_d_path. It becomes path_offset = -(raw * 0.001) meters and is added to the model path y coordinate; with LiveTune enabled it reloads about once per second.",
    "lane_planner.get_d_path가 읽습니다. path_offset = -(raw * 0.001) m 로 변환되어 모델 path의 y 좌표에 더해지며, LiveTune이 켜져 있으면 약 1초마다 다시 읽습니다.",
    "Raw integer millimeters. This moves the planned path, not the camera image.",
    "원시 정수 mm. 카메라 이미지가 아니라 계획 경로를 이동시킵니다.",
  ),
  "SteerActuatorDelayAdj": doc(
    "Read in Hyundai CarInterface when CarParams are built. It becomes ret.steerActuatorDelay = raw * 0.01 seconds.",
    "Hyundai CarInterface가 CarParams 생성 시 읽습니다. ret.steerActuatorDelay = raw * 0.01초로 변환됩니다.",
    "36 means 0.36 s.",
    "36은 0.36초입니다.",
  ),
  "TireStiffnessFactorAdj": doc(
    "Read by Hyundai CarInterface and locationd/paramsd. It becomes a tire stiffness scale of raw * 0.01.",
    "Hyundai CarInterface와 locationd/paramsd가 읽습니다. raw * 0.01의 타이어 강성 scale로 변환됩니다.",
    "85 means 0.85x.",
    "85는 0.85배입니다.",
  ),
  "SteerLimitTimerAdj": doc(
    "Read in Hyundai CarInterface when CarParams are built. It becomes ret.steerLimitTimer = raw * 0.01 seconds.",
    "Hyundai CarInterface가 CarParams 생성 시 읽습니다. ret.steerLimitTimer = raw * 0.01초로 변환됩니다.",
    "100 means 1.00 s.",
    "100은 1.00초입니다.",
  ),
  "OpkrLiveSteerRatio": doc(
    "Read by controlsd at startup and then about once per second. When enabled, controlsd can adjust steerRatio between the base and max values using speed/angle logic.",
    "controlsd가 시작 시와 이후 약 1초마다 읽습니다. 켜면 speed/angle 로직에 따라 base와 max 사이에서 steerRatio를 조정할 수 있습니다.",
    "0=fixed CarParams steerRatio, 1=live steer-ratio adjustment.",
    "0=CarParams steerRatio 고정, 1=live steer-ratio 조정.",
  ),
  "LiveSteerRatioPercent": doc(
    "Read by controlsd with OpkrLiveSteerRatio. It biases the live steer-ratio calculation as a percentage adjustment.",
    "controlsd가 OpkrLiveSteerRatio와 함께 읽습니다. live steer-ratio 계산에 percent 보정으로 들어갑니다.",
    "Signed percent value.",
    "부호 있는 percent 값.",
  ),
  "SteerRatioAdj": doc(
    "Read in Hyundai CarInterface when CarParams are built. It becomes ret.steerRatio = raw * 0.01.",
    "Hyundai CarInterface가 CarParams 생성 시 읽습니다. ret.steerRatio = raw * 0.01로 변환됩니다.",
    "1550 means steer ratio 15.50.",
    "1550은 steer ratio 15.50입니다.",
  ),
  "SteerRatioMaxAdj": doc(
    "Read by controlsd. It becomes steerRatio_Max = raw * 0.01 and forms the upper bound of the live steer-ratio range.",
    "controlsd가 읽습니다. steerRatio_Max = raw * 0.01로 변환되어 live steer-ratio 범위의 상한이 됩니다.",
    "1750 means max steer ratio 17.50.",
    "1750은 최대 steer ratio 17.50입니다.",
  ),
  "OpkrVariableSteerMax": doc(
    "Read by Hyundai carcontroller. When enabled above about 8.3 m/s, steerMax is interpolated from modelSpeed using SteerMaxAdj as the low-speed max and SteerMaxBaseAdj as the higher-speed value.",
    "Hyundai carcontroller가 읽습니다. 켜면 약 8.3 m/s 이상에서 modelSpeed를 기준으로 SteerMaxAdj를 저속 최대값, SteerMaxBaseAdj를 고속쪽 값으로 보간해 steerMax를 정합니다.",
    "0=always use SteerMaxBaseAdj, 1=interpolate variable steerMax.",
    "0=항상 SteerMaxBaseAdj 사용, 1=variable steerMax 보간 사용.",
  ),
  "OpkrVariableSteerDelta": doc(
    "Read by Hyundai carcontroller. When enabled above about 8.3 m/s, steer delta-up/down limits are interpolated from modelSpeed using the base and max delta settings.",
    "Hyundai carcontroller가 읽습니다. 켜면 약 8.3 m/s 이상에서 modelSpeed를 기준으로 steer delta-up/down 제한을 base/max delta 설정 사이에서 보간합니다.",
    "0=always use base delta values, 1=interpolate variable delta values.",
    "0=항상 base delta 사용, 1=variable delta 보간 사용.",
  ),
  "OpkrLiveTunePanelEnable": doc(
    "Read by lane_planner and lateral controllers. When enabled, lane offsets reload about once per second and PID/INDI/LQR/Torque gains reload periodically while driving.",
    "lane_planner와 lateral controller들이 읽습니다. 켜면 lane offset은 약 1초마다, PID/INDI/LQR/Torque gain은 주행 중 주기적으로 다시 읽습니다.",
    "0=values apply at component start, 1=selected tuning values reload live.",
    "0=구성요소 시작 시 적용, 1=선택된 튜닝값을 live reload.",
  ),
  "PidKp": doc(
    "Used by PID lateral tuning and live_tune. It becomes steerKpV = raw * 0.01, with PID breakpoints [0, 9].",
    "PID lateral tuning과 live_tune이 사용합니다. steerKpV = raw * 0.01로 변환되며 PID breakpoint [0, 9]에 들어갑니다.",
    "25 means Kp 0.25.",
    "25는 Kp 0.25입니다.",
  ),
  "PidKi": doc(
    "Used by PID lateral tuning and live_tune. It becomes steerKiV = raw * 0.001.",
    "PID lateral tuning과 live_tune이 사용합니다. steerKiV = raw * 0.001로 변환됩니다.",
    "40 means Ki 0.040.",
    "40은 Ki 0.040입니다.",
  ),
  "PidKd": doc(
    "Used by PID lateral tuning and live_tune. It becomes steerKdV = raw * 0.01.",
    "PID lateral tuning과 live_tune이 사용합니다. steerKdV = raw * 0.01로 변환됩니다.",
    "150 means Kd 1.50.",
    "150은 Kd 1.50입니다.",
  ),
  "PidKf": doc(
    "Used by PID lateral tuning and live_tune. It becomes steerKf = raw * 0.00001.",
    "PID lateral tuning과 live_tune이 사용합니다. steerKf = raw * 0.00001로 변환됩니다.",
    "7 means Kf 0.00007.",
    "7은 Kf 0.00007입니다.",
  ),
  "InnerLoopGain": doc(
    "Used by INDI lateral tuning and live_tune. It becomes innerLoopGain = raw * 0.1 and affects steering-rate error correction.",
    "INDI lateral tuning과 live_tune이 사용합니다. innerLoopGain = raw * 0.1로 변환되며 steering-rate error 보정에 영향을 줍니다.",
    "35 means 3.5.",
    "35는 3.5입니다.",
  ),
  "OuterLoopGain": doc(
    "Used by INDI lateral tuning and live_tune. It becomes outerLoopGain = raw * 0.1 and affects steering-angle/lane-centering response.",
    "INDI lateral tuning과 live_tune이 사용합니다. outerLoopGain = raw * 0.1로 변환되며 steering-angle/lane-centering 응답에 영향을 줍니다.",
    "20 means 2.0.",
    "20은 2.0입니다.",
  ),
  "TimeConstant": doc(
    "Used by INDI lateral tuning and live_tune. It becomes timeConstant = raw * 0.1 and controls how quickly previous actuation decays.",
    "INDI lateral tuning과 live_tune이 사용합니다. timeConstant = raw * 0.1로 변환되며 이전 조향 출력의 decay 속도에 영향을 줍니다.",
    "14 means 1.4.",
    "14는 1.4입니다.",
  ),
  "ActuatorEffectiveness": doc(
    "Used by INDI lateral tuning and live_tune. It becomes actuatorEffectiveness = raw * 0.1; higher values reduce effective actuation strength in the INDI model.",
    "INDI lateral tuning과 live_tune이 사용합니다. actuatorEffectiveness = raw * 0.1로 변환되며, INDI 모델에서는 값이 높을수록 유효 조향 강도가 작아지는 방향입니다.",
    "20 means 2.0.",
    "20은 2.0입니다.",
  ),
  "Scale": doc(
    "Used by LQR lateral tuning and live_tune. It is passed as the LQR scale value without decimal scaling.",
    "LQR lateral tuning과 live_tune이 사용합니다. 별도 소수 변환 없이 LQR scale 값으로 들어갑니다.",
    "1500 means scale 1500.",
    "1500은 scale 1500입니다.",
  ),
  "LqrKi": doc(
    "Used by LQR lateral tuning and live_tune. It becomes LQR ki = raw * 0.001.",
    "LQR lateral tuning과 live_tune이 사용합니다. LQR ki = raw * 0.001로 변환됩니다.",
    "16 means ki 0.016.",
    "16은 ki 0.016입니다.",
  ),
  "DcGain": doc(
    "Used by LQR lateral tuning and live_tune. It becomes dcGain = raw * 0.00001.",
    "LQR lateral tuning과 live_tune이 사용합니다. dcGain = raw * 0.00001로 변환됩니다.",
    "265 means dcGain 0.00265.",
    "265는 dcGain 0.00265입니다.",
  ),
  "TorqueMaxLatAccel": doc(
    "Used by Torque tuning and live_tune. It becomes max_lat_accel = raw * 0.1, then TorqueKp/Kf/Ki are divided by this value.",
    "Torque tuning과 live_tune이 사용합니다. max_lat_accel = raw * 0.1로 변환된 뒤 TorqueKp/Kf/Ki가 이 값으로 나뉩니다.",
    "27 means 2.7 m/s^2.",
    "27은 2.7 m/s^2입니다.",
  ),
  "TorqueKp": doc(
    "Used by Torque tuning and live_tune. Raw is first multiplied by 0.1, then normalized by TorqueMaxLatAccel.",
    "Torque tuning과 live_tune이 사용합니다. raw에 먼저 0.1을 곱한 뒤 TorqueMaxLatAccel로 정규화합니다.",
    "Effective kp = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 kp = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueKf": doc(
    "Used by Torque tuning and live_tune. Raw is first multiplied by 0.1, then normalized by TorqueMaxLatAccel as feedforward gain.",
    "Torque tuning과 live_tune이 사용합니다. raw에 먼저 0.1을 곱한 뒤 TorqueMaxLatAccel로 정규화되어 feedforward gain이 됩니다.",
    "Effective kf = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 kf = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueKi": doc(
    "Used by Torque tuning and live_tune. Raw is first multiplied by 0.1, then normalized by TorqueMaxLatAccel as integral gain.",
    "Torque tuning과 live_tune이 사용합니다. raw에 먼저 0.1을 곱한 뒤 TorqueMaxLatAccel로 정규화되어 integral gain이 됩니다.",
    "Effective ki = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 ki = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueFriction": doc(
    "Used by Torque tuning and live_tune as friction compensation. It becomes raw * 0.001.",
    "Torque tuning과 live_tune이 friction compensation으로 사용합니다. raw * 0.001로 변환됩니다.",
    "65 means 0.065.",
    "65는 0.065입니다.",
  ),
  "TorqueUseAngle": doc(
    "Used by Torque tuning and live_tune. It selects whether steering angle feedback is used in torque lateral control.",
    "Torque tuning과 live_tune이 사용합니다. torque lateral control에서 steering angle feedback을 사용할지 선택합니다.",
    "0=do not use steering angle, 1=use steering angle.",
    "0=steering angle 미사용, 1=steering angle 사용.",
  ),
  "TorqueAngDeadZone": doc(
    "Used by Torque tuning and live_tune. It becomes steeringAngleDeadzoneDeg = raw * 0.1 degrees.",
    "Torque tuning과 live_tune이 사용합니다. steeringAngleDeadzoneDeg = raw * 0.1도로 변환됩니다.",
    "10 means 1.0 degree.",
    "10은 1.0도입니다.",
  ),
  "CustomTREnabled": doc(
    "Read by long MPC. When enabled, the desired following time gap TR is built from CruiseGap values and/or DynamicTR interpolation; when disabled, TR falls back to 1.45.",
    "long MPC가 읽습니다. 켜면 CruiseGap 값과 DynamicTR 보간으로 추종 시간 간격 TR을 만들고, 끄면 TR은 1.45로 돌아갑니다.",
    "0=fixed 1.45 TR, 1=custom TR logic.",
    "0=고정 TR 1.45, 1=custom TR 로직.",
  ),
  "CruiseGap1": doc(
    "Read by long MPC as the desired following time gap for cruise gap level 1. Raw value is multiplied by 0.1 seconds.",
    "long MPC가 cruise gap 1단계의 목표 추종 시간 간격으로 읽습니다. raw 값에 0.1초를 곱합니다.",
    "12 means 1.2 s.",
    "12는 1.2초입니다.",
  ),
  "CruiseGap2": doc(
    "Read by long MPC as the desired following time gap for cruise gap level 2. Raw value is multiplied by 0.1 seconds.",
    "long MPC가 cruise gap 2단계의 목표 추종 시간 간격으로 읽습니다. raw 값에 0.1초를 곱합니다.",
    "13 means 1.3 s.",
    "13은 1.3초입니다.",
  ),
  "CruiseGap3": doc(
    "Read by long MPC as the desired following time gap for cruise gap level 3. Raw value is multiplied by 0.1 seconds.",
    "long MPC가 cruise gap 3단계의 목표 추종 시간 간격으로 읽습니다. raw 값에 0.1초를 곱합니다.",
    "14 means 1.4 s.",
    "14는 1.4초입니다.",
  ),
  "CruiseGap4": doc(
    "Read by long MPC as the desired following time gap for cruise gap level 4. Raw value is multiplied by 0.1 seconds.",
    "long MPC가 cruise gap 4단계의 목표 추종 시간 간격으로 읽습니다. raw 값에 0.1초를 곱합니다.",
    "16 means 1.6 s.",
    "16은 1.6초입니다.",
  ),
  "DynamicTRGap": doc(
    "Read by long MPC. It selects which cruise-gap level is replaced by speed-interpolated DynamicTRSet values.",
    "long MPC가 읽습니다. 어떤 cruise-gap 단계에 속도 보간된 DynamicTRSet 값을 넣을지 선택합니다.",
    "0=use CruiseGap1-4 directly; 1-4=replace that gap level with DynamicTR.",
    "0=CruiseGap1-4 직접 사용, 1-4=해당 gap 단계에 DynamicTR 적용.",
  ),
  "DynamicTRSpd": doc(
    "Read by long MPC as speed breakpoints for dynamic following time. It must align one-to-one with DynamicTRSet.",
    "long MPC가 dynamic following time의 속도 breakpoint로 읽습니다. DynamicTRSet과 개수가 1:1로 맞아야 합니다.",
    "Comma-separated speeds in kph or mph according to IsMetric.",
    "IsMetric 설정에 따른 km/h 또는 mph 속도값을 comma로 구분합니다.",
  ),
  "DynamicTRSet": doc(
    "Read by long MPC as following-time values interpolated over DynamicTRSpd. It must align one-to-one with DynamicTRSpd.",
    "long MPC가 DynamicTRSpd 위에서 보간할 추종 시간 값으로 읽습니다. DynamicTRSpd와 개수가 1:1로 맞아야 합니다.",
    "Comma-separated seconds.",
    "초 단위 값을 comma로 구분합니다.",
  ),
  "RadarLongHelper": doc(
    "Original OPKR longitudinal mode selector. The Qt menu maps it to Vision Only, Radar Only, or OPKR; current K230 validation should treat changes as safety-sensitive.",
    "기존 OPKR longitudinal 모드 선택값입니다. Qt 메뉴에서는 Vision Only, Radar Only, OPKR로 매핑되며, K230 검증에서는 안전 관련 변경으로 취급해야 합니다.",
    "0=Vision Only, 1=Radar Only, 2=OPKR.",
    "0=Vision Only, 1=Radar Only, 2=OPKR.",
  ),
  "StoppingDist": doc(
    "Read by longcontrol. It becomes stopping_dist = raw * 0.1 meters for stop-distance adjustment logic.",
    "longcontrol이 읽습니다. stop-distance 보정 로직에서 stopping_dist = raw * 0.1 m로 변환됩니다.",
    "38 means 3.8 m.",
    "38은 3.8 m입니다.",
  ),
  "E2ELong": doc(
    "Read by long MPC and controlsd. It switches the longitudinal planner toward the end-to-end long policy and raises an OPKR alert once when enabled.",
    "long MPC와 controlsd가 읽습니다. longitudinal planner를 end-to-end long 정책 쪽으로 전환하고, 켜졌을 때 OPKR alert를 한 번 발생시킵니다.",
    "0=lead/radar-style policy, 1=end-to-end longitudinal policy.",
    "0=lead/radar 계열 정책, 1=end-to-end longitudinal 정책.",
  ),
  "ShowStopLine": doc(
    "Read by long MPC. When enabled, model.stopLine probability can be used as a stopping obstacle.",
    "long MPC가 읽습니다. 켜면 model.stopLine probability를 정지 장애물로 사용할 수 있습니다.",
    "0=ignore stop-line model output, 1=use stop-line output.",
    "0=stop-line 모델 출력 무시, 1=stop-line 출력 사용.",
  ),
  "MultipleLateralUse": doc(
    "Read by LatControlATOM. It selects whether ATOM chooses sub-controllers by fixed method, steering angle interpolation, or speed interpolation.",
    "LatControlATOM이 읽습니다. ATOM이 하위 controller를 고정 방식, 조향각 보간, 속도 보간 중 어떤 방식으로 고를지 선택합니다.",
    "2=angle interpolation, 3=speed interpolation in this code path.",
    "이 코드 경로에서 2=조향각 보간, 3=속도 보간입니다.",
  ),
  "MultipleLateralSpd": doc(
    "Read by LatControlATOM as speed breakpoints for speed-based multi-lateral interpolation.",
    "LatControlATOM이 속도 기반 multi-lateral 보간의 speed breakpoint로 읽습니다.",
    "Comma-separated speed breakpoints; length must match MultipleLateralOpS.",
    "속도 breakpoint를 comma로 구분하며 MultipleLateralOpS와 개수가 맞아야 합니다.",
  ),
  "MultipleLateralOpS": doc(
    "Read by LatControlATOM as controller method ids used at MultipleLateralSpd breakpoints.",
    "LatControlATOM이 MultipleLateralSpd breakpoint에서 사용할 controller method id로 읽습니다.",
    "Method ids: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
    "Method id: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
  ),
  "MultipleLateralAng": doc(
    "Read by LatControlATOM as steering-angle breakpoints for angle-based multi-lateral interpolation.",
    "LatControlATOM이 조향각 기반 multi-lateral 보간의 angle breakpoint로 읽습니다.",
    "Comma-separated steering-angle breakpoints; length must match MultipleLateralOpA.",
    "조향각 breakpoint를 comma로 구분하며 MultipleLateralOpA와 개수가 맞아야 합니다.",
  ),
  "MultipleLateralOpA": doc(
    "Read by LatControlATOM as controller method ids used at MultipleLateralAng breakpoints.",
    "LatControlATOM이 MultipleLateralAng breakpoint에서 사용할 controller method id로 읽습니다.",
    "Method ids: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
    "Method id: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
  ),
}

DRIVING_EFFECT_DOCS: Dict[str, HelpDoc] = {
  "OpkrAutoResume": doc(
    "Hyundai carcontroller uses this while SCC is stopped. When the lead car starts moving, it sends RES_ACCEL button messages so the car can depart without the driver pressing RES. Turning it off makes stop-and-go wait for driver input.",
    "Hyundai carcontroller가 SCC 정차 중 사용합니다. 앞차가 움직이기 시작하면 RES_ACCEL 버튼 메시지를 보내 운전자가 RES를 누르지 않아도 재출발할 수 있게 합니다. 끄면 stop-and-go 출발을 운전자 입력에 맡깁니다.",
    "0=manual resume, 1=auto RES burst at standstill.",
    "0=수동 재출발, 1=정차 중 자동 RES burst.",
  ),
  "RESCountatStandstill": doc(
    "Number of RES_ACCEL messages sent in each standstill auto-resume burst. Higher values make stock SCC more likely to accept resume, but also spam more cruise-button CAN messages. Lower values are gentler but can miss the resume.",
    "정차 자동 재출발 때 한 번에 보내는 RES_ACCEL 메시지 개수입니다. 값을 높이면 순정 SCC가 재출발을 받아들일 가능성이 커지지만 cruise 버튼 CAN 메시지도 더 많이 나갑니다. 낮추면 부드럽지만 재출발을 놓칠 수 있습니다.",
    "Raw message count per burst.",
    "한 번의 burst에 보낼 메시지 개수입니다.",
  ),
  "CruiseGapAdjust": doc(
    "At standstill, the Hyundai controller can temporarily move the stock SCC gap to 1 for a quicker launch, then restore the saved gap after departure. Turning it off keeps the driver-selected gap unchanged while stopped.",
    "정차 중 Hyundai controller가 순정 SCC gap을 임시로 1단으로 바꿔 출발 반응을 빠르게 하고, 출발 후 저장된 gap으로 복귀시킬 수 있습니다. 끄면 정차 중에도 운전자가 고른 gap을 유지합니다.",
    "0=keep current gap, 1=temporary gap 1 at standstill.",
    "0=현재 gap 유지, 1=정차 중 임시 gap 1 사용.",
  ),
  "CruiseGapBySpdOn": doc(
    "Hyundai carcontroller periodically presses the GAP button to match the current speed band. Turning it on makes following distance change automatically with speed; turning it off leaves stock SCC gap control to the driver.",
    "Hyundai carcontroller가 현재 속도 구간에 맞게 GAP 버튼 메시지를 주기적으로 보냅니다. 켜면 속도에 따라 추종 거리가 자동으로 바뀌고, 끄면 순정 SCC gap 선택을 운전자에게 맡깁니다.",
    "0=manual stock gap, 1=use CruiseGapBySpdSpd/Gaps.",
    "0=수동 순정 gap, 1=CruiseGapBySpdSpd/Gaps 사용.",
  ),
  "CruiseGapBySpdSpd": doc(
    "Speed breakpoints for speed-based cruise gap. Crossing a breakpoint changes which target gap the controller requests; the controller uses a small hysteresis to avoid rapid toggling.",
    "속도 기반 cruise gap의 속도 breakpoint입니다. 속도가 breakpoint를 넘으면 controller가 요청하는 목표 gap이 바뀌며, 잦은 왕복을 막기 위해 작은 hysteresis를 사용합니다.",
    "Comma-separated speed breakpoints. This list should have one fewer item than CruiseGapBySpdGap.",
    "comma로 구분한 속도 breakpoint입니다. CruiseGapBySpdGap보다 항목이 하나 적어야 합니다.",
  ),
  "CruiseGapBySpdGap": doc(
    "Target SCC gap levels for the speed bands. Lower gap values follow closer and react later to the lead car; higher gap values leave more distance and feel more conservative.",
    "속도 구간별 목표 SCC gap 단계입니다. 낮은 gap은 앞차에 더 가깝게 따라가고 반응이 늦게 느껴질 수 있으며, 높은 gap은 거리를 더 남겨 보수적으로 느껴집니다.",
    "Comma-separated gap levels, usually four values for three speed breakpoints.",
    "comma로 구분한 gap 단계입니다. 보통 속도 breakpoint 3개에 gap 4개를 둡니다.",
  ),
  "StandstillResumeAlt": doc(
    "Alternative standstill resume timing for cars that reject the default resume burst. It sends shorter randomized RES bursts, which can be more compatible on some Hyundai/Genesis SCC variants.",
    "기본 resume burst를 잘 받지 않는 차량을 위한 정차 재출발 대체 타이밍입니다. 더 짧고 약간 랜덤한 RES burst를 보내 일부 Hyundai/Genesis SCC 변형에서 호환성이 좋아질 수 있습니다.",
    "0=default standstill resume, 1=alternative burst timing.",
    "0=기본 정차 재출발, 1=대체 burst 타이밍.",
  ),
  "DepartChimeAtResume": doc(
    "Adds a depart/resume chime when auto resume triggers. It changes driver feedback only; it does not change acceleration, braking, or path planning.",
    "자동 재출발이 걸릴 때 출발 알림음을 더합니다. 운전자 피드백만 바꾸며 가속, 제동, 경로 계획은 바꾸지 않습니다.",
    "0=no chime, 1=play chime on resume.",
    "0=알림음 없음, 1=재출발 시 알림음.",
  ),
  "OpkrVariableCruise": doc(
    "controlsd and Hyundai controller use this to adjust the stock cruise set speed by sending cruise-button messages toward an internally computed target. Turning it on allows automatic set-speed changes for distance, curve, and camera-speed logic; turning it off keeps set speed more manual.",
    "controlsd와 Hyundai controller가 내부 목표 속도에 맞추기 위해 cruise 버튼 메시지로 순정 cruise set speed를 조정할 때 사용합니다. 켜면 거리, 커브, 카메라 속도 로직에 따라 set speed가 자동으로 바뀌고, 끄면 set speed가 더 수동적으로 유지됩니다.",
    "0=do not spam cruise buttons for variable cruise, 1=allow automatic button control.",
    "0=variable cruise용 버튼 전송 안 함, 1=자동 버튼 제어 허용.",
  ),
  "VarCruiseSpeedFactor": doc(
    "Aggressiveness for variable-cruise button control. Higher values let the controller chase target speed faster with more button events; lower values make target-speed changes slower and calmer.",
    "variable cruise 버튼 제어의 적극성입니다. 값을 높이면 더 많은 버튼 이벤트로 목표 속도를 빠르게 따라가고, 낮추면 목표 속도 변화가 느리고 차분해집니다.",
    "Raw integer factor used by the cruise-button logic.",
    "cruise 버튼 로직이 사용하는 원시 정수 계수입니다.",
  ),
  "CruiseSetwithRoadLimitSpeedEnabled": doc(
    "controlsd allows external roadLimitSpeed data to set or bias the cruise target. Turning it on can make the set speed follow road-speed information; turning it off ignores that source.",
    "controlsd가 외부 roadLimitSpeed 데이터로 cruise 목표를 설정하거나 보정할 수 있게 합니다. 켜면 set speed가 도로 제한 속도 정보를 따라갈 수 있고, 끄면 이 입력을 무시합니다.",
    "0=ignore roadLimitSpeed for cruise set, 1=use roadLimitSpeed.",
    "0=cruise set에 roadLimitSpeed 미사용, 1=roadLimitSpeed 사용.",
  ),
  "CruiseSetwithRoadLimitSpeedOffset": doc(
    "Offset added when cruise speed is set from roadLimitSpeed. Positive values request a faster set speed than the road limit; negative values request a slower one.",
    "roadLimitSpeed로 cruise speed를 정할 때 더하는 offset입니다. 양수는 제한 속도보다 빠른 set speed를 요청하고, 음수는 더 느린 set speed를 요청합니다.",
    "Stored in the same speed unit used by the road-limit path.",
    "road-limit 경로가 쓰는 속도 단위와 같은 단위로 저장됩니다.",
  ),
  "CruiseStatemodeSelInit": doc(
    "Initial cruise mode used by the OPKR speed-control path. It selects which automatic target-speed sources are allowed. Modes with distance, curve, or safety-camera sources can slow down more often; OP mode leaves more control to the base cruise behavior.",
    "OPKR 속도 제어 경로의 초기 cruise mode입니다. 어떤 자동 목표 속도 source를 허용할지 고릅니다. 거리, 커브, 안전카메라 source가 포함된 mode는 더 자주 감속할 수 있고, OP mode는 기본 cruise 동작에 더 많이 맡깁니다.",
    "0=OP mode, 1=distance+curve, 2=distance, 3=curve, 4=one-way, 5=safety camera.",
    "0=OP mode, 1=거리+커브, 2=거리, 3=커브, 4=일방통행, 5=안전카메라.",
  ),
  "OpkrLaneChangeSpeed": doc(
    "desire_helper uses this as the minimum speed for lane change. Values below 1 disable lane changes. Raising it prevents lane changes at lower speed; lowering it allows lane changes earlier.",
    "desire_helper가 차선 변경 최소 속도로 사용합니다. 1보다 낮으면 차선 변경을 비활성화합니다. 값을 높이면 저속 차선 변경을 막고, 낮추면 더 이른 속도에서 차선 변경을 허용합니다.",
    "Speed follows IsMetric: kph when metric, mph when imperial.",
    "속도 단위는 IsMetric을 따릅니다. metric이면 km/h, imperial이면 mph입니다.",
  ),
  "OpkrAutoLaneChangeDelay": doc(
    "Delay after the blinker turns on before lane change may start without driver steering torque. Higher delay gives the driver more time to cancel; 0 requires steering torque instead of automatic start.",
    "방향지시등을 켠 뒤 운전자 조향 토크 없이 차선 변경을 시작하기까지의 지연입니다. 값을 높이면 취소할 시간이 길어지고, 0은 자동 시작 대신 운전자 조향 토크를 요구합니다.",
    "Code mapping: 0=torque only, 1=0.2 s, 2=0.5 s, 3=1.0 s, 4=1.5 s, 5+=2.0 s.",
    "코드 매핑: 0=토크 필요, 1=0.2초, 2=0.5초, 3=1.0초, 4=1.5초, 5 이상=2.0초.",
  ),
  "LCTimingFactorEnable": doc(
    "Enables speed-dependent lane-change timing factors. When enabled, the factor controls how quickly lane-line probability fades during a lane change; higher factors make the maneuver progress faster.",
    "속도별 차선 변경 timing factor를 켭니다. 켜면 차선 변경 중 차선선 probability가 사라지는 속도를 factor가 제어하며, factor가 높을수록 차선 변경 진행이 빨라집니다.",
    "0=use default lane-change timing, 1=use LCTimingFactor values.",
    "0=기본 차선 변경 타이밍, 1=LCTimingFactor 값 사용.",
  ),
  "LCTimingFactorUD": doc(
    "Original UI mode selector for lane-change timing. In the current Python desire_helper path, LCTimingFactorEnable and the numeric speed factors are the values that directly change driving.",
    "차선 변경 timing의 기존 UI mode 선택값입니다. 현재 Python desire_helper 경로에서는 LCTimingFactorEnable과 숫자 speed factor들이 실제 주행을 직접 바꿉니다.",
    "Keep User Defined when editing the factor values.",
    "factor 값을 직접 조정할 때는 User Defined로 두는 것이 맞습니다.",
  ),
  "LCTimingFactor30": doc(
    "Lane-change timing factor around 30 speed units. Higher values fade lane lines faster and shorten the lane-change transition; lower values keep the transition smoother and longer.",
    "약 30 속도 구간의 차선 변경 timing factor입니다. 값을 높이면 차선선 fade가 빨라져 차선 변경 전환이 짧아지고, 낮추면 더 부드럽고 길게 진행됩니다.",
    "Raw value is multiplied by 0.01 in desire_helper.",
    "desire_helper에서 raw 값에 0.01을 곱합니다.",
  ),
  "LCTimingFactor60": doc(
    "Lane-change timing factor around 60 speed units. Higher values make highway-speed lane changes complete more decisively; lower values make them slower.",
    "약 60 속도 구간의 차선 변경 timing factor입니다. 값을 높이면 고속 차선 변경이 더 단호하게 끝나고, 낮추면 더 천천히 진행됩니다.",
    "Raw value is multiplied by 0.01 in desire_helper.",
    "desire_helper에서 raw 값에 0.01을 곱합니다.",
  ),
  "LCTimingFactor80": doc(
    "Lane-change timing factor around 80 speed units. Raising it makes lane changes at this speed fade lane-line confidence faster; lowering it makes lateral transition more gradual.",
    "약 80 속도 구간의 차선 변경 timing factor입니다. 높이면 이 속도에서 차선선 confidence가 더 빨리 줄고, 낮추면 횡방향 전환이 더 완만해집니다.",
    "Raw value is multiplied by 0.01 in desire_helper.",
    "desire_helper에서 raw 값에 0.01을 곱합니다.",
  ),
  "LCTimingFactor110": doc(
    "Lane-change timing factor around 110 speed units. It mostly affects high-speed lane changes; higher values are quicker, lower values are more gradual.",
    "약 110 속도 구간의 차선 변경 timing factor입니다. 주로 고속 차선 변경에 영향을 주며, 높이면 빠르고 낮추면 완만합니다.",
    "Raw value is multiplied by 0.01 in desire_helper.",
    "desire_helper에서 raw 값에 0.01을 곱합니다.",
  ),
  "LeftCurvOffsetAdj": doc(
    "lane_planner applies this only in left curves above about 8 m/s when the lane geometry condition matches. Larger absolute values bias the planned lane center farther during left curves; the sign chooses which side by the OPKR convention.",
    "lane_planner가 약 8 m/s 이상 좌커브에서 차선 geometry 조건이 맞을 때만 적용합니다. 절대값을 키우면 좌커브 중 계획 차선 중심을 더 많이 치우치게 하며, 부호는 OPKR 규칙에 따라 이동 방향을 고릅니다.",
    "Not direct meters. The code scales abs(raw) by lane_differ * 0.05 and caps the lane_differ input.",
    "직접 m 값은 아닙니다. 코드가 abs(raw)에 lane_differ * 0.05를 곱하고 lane_differ 입력을 제한합니다.",
  ),
  "RightCurvOffsetAdj": doc(
    "lane_planner applies this only in right curves above about 8 m/s when the lane geometry condition matches. Larger absolute values bias the planned lane center farther during right curves; the sign chooses which side by the OPKR convention.",
    "lane_planner가 약 8 m/s 이상 우커브에서 차선 geometry 조건이 맞을 때만 적용합니다. 절대값을 키우면 우커브 중 계획 차선 중심을 더 많이 치우치게 하며, 부호는 OPKR 규칙에 따라 이동 방향을 고릅니다.",
    "Not direct meters. The code scales abs(raw) by lane_differ * 0.05 and caps the lane_differ input.",
    "직접 m 값은 아닙니다. 코드가 abs(raw)에 lane_differ * 0.05를 곱하고 lane_differ 입력을 제한합니다.",
  ),
  "OpkrBlindSpotDetect": doc(
    "The confirmed reader is the UI drawing path. It shows blind-spot status on screen when the carState blind-spot signals are present. Lane-change blocking itself uses the carState blind-spot values, not this display toggle.",
    "현재 확인된 reader는 UI drawing 경로입니다. carState의 blind-spot 신호가 있을 때 화면에 상태를 표시합니다. 차선 변경 차단 자체는 이 표시 토글이 아니라 carState blind-spot 값을 사용합니다.",
    "0=hide BSM display, 1=show BSM display.",
    "0=BSM 표시 숨김, 1=BSM 표시.",
  ),
  "OpkrSteerMethod": doc(
    "Hyundai smooth_steer mode selector. Normal applies requested torque directly through the usual limits; Smooth scales torque down when steering angle or driver steering conditions say it should fade.",
    "Hyundai smooth_steer mode 선택값입니다. Normal은 일반 제한을 거쳐 요청 토크를 적용하고, Smooth는 조향각 또는 운전자 조향 조건에 따라 토크를 줄여 fade시킵니다.",
    "0=Normal, 1=Smooth.",
    "0=Normal, 1=Smooth.",
  ),
  "OpkrMaxAngleLimit": doc(
    "Hyundai controller uses this as the steering-angle gate for LKAS torque when the LKAS-fault avoidance path is not active. Lower effective limits cut steering assist earlier in sharp turns; higher limits allow assist deeper into steering angle.",
    "LKAS fault avoidance 경로가 꺼져 있을 때 Hyundai controller가 LKAS 토크 허용 조향각 gate로 사용합니다. 유효 제한을 낮추면 급커브에서 보조가 더 일찍 끊기고, 높이면 더 큰 조향각까지 보조를 허용합니다.",
    "Degrees. Keep enough margin to avoid stock LKAS faults.",
    "도 단위입니다. 순정 LKAS fault를 피할 여유를 남겨야 합니다.",
  ),
  "OpkrMaxSteeringAngle": doc(
    "Smooth steering threshold. Above this steering angle, smooth_steer starts reducing allowed torque according to the max-angle wait values. Higher values delay torque fade; lower values fade earlier.",
    "Smooth steering 임계 조향각입니다. 이 각도 이상에서는 smooth_steer가 max-angle wait 값에 따라 허용 토크를 줄입니다. 높이면 torque fade가 늦고, 낮추면 더 일찍 fade됩니다.",
    "Degrees.",
    "도 단위입니다.",
  ),
  "OpkrMaxDriverAngleWait": doc(
    "Smooth steering fade rate when both the steering angle is above OpkrMaxSteeringAngle and the driver is steering. Higher values reduce assist faster; lower values hold assist longer.",
    "조향각이 OpkrMaxSteeringAngle보다 크고 운전자가 조향 중일 때 Smooth steering의 fade 속도입니다. 높이면 보조가 더 빨리 줄고, 낮추면 더 오래 유지됩니다.",
    "Seconds subtracted from the smooth-steer timer each control step.",
    "각 control step에서 smooth-steer timer에서 빼는 초 단위 값입니다.",
  ),
  "OpkrMaxSteerAngleWait": doc(
    "Smooth steering fade rate when the steering angle is above OpkrMaxSteeringAngle without driver torque. Higher values reduce assist faster in high-angle turns; lower values keep assist longer.",
    "운전자 토크 없이 조향각이 OpkrMaxSteeringAngle보다 클 때 Smooth steering fade 속도입니다. 높이면 큰 조향각에서 보조가 빨리 줄고, 낮추면 더 오래 유지됩니다.",
    "Seconds subtracted from the smooth-steer timer each control step.",
    "각 control step에서 smooth-steer timer에서 빼는 초 단위 값입니다.",
  ),
  "OpkrDriverAngleWait": doc(
    "Smooth steering fade rate when the driver is steering below the max-angle threshold. Higher values give driver torque priority sooner; lower values keep openpilot torque blended longer.",
    "max-angle 임계값 아래에서 운전자가 조향 중일 때 Smooth steering fade 속도입니다. 높이면 운전자 조향을 더 빨리 우선하고, 낮추면 openpilot 토크를 더 오래 섞습니다.",
    "Seconds subtracted from the smooth-steer timer each control step.",
    "각 control step에서 smooth-steer timer에서 빼는 초 단위 값입니다.",
  ),
  "OpkrSteerAngleCorrection": doc(
    "Hyundai carstate adds this correction to the reported steering angle. It shifts the angle seen by planners/controllers, so a wrong sign or large value can bias lane centering and steering-limit decisions.",
    "Hyundai carstate가 보고되는 steering angle에 이 보정값을 더합니다. planner/controller가 보는 조향각을 바꾸므로 부호가 틀리거나 값이 크면 차선 중앙 유지와 steering limit 판단이 치우칠 수 있습니다.",
    "Raw value is multiplied by 0.1 degrees.",
    "raw 값에 0.1도를 곱합니다.",
  ),
  "OpkrTurnSteeringDisable": doc(
    "Hyundai controller temporarily disables LKAS torque when a turn signal is on below the lane-change minimum speed. Turning it on prevents steering assist from fighting low-speed turns; turning it off keeps assist available.",
    "차선 변경 최소 속도보다 낮고 방향지시등이 켜졌을 때 Hyundai controller가 LKAS 토크를 임시로 끕니다. 켜면 저속 회전에서 조향 보조가 운전자 조작을 방해하지 않고, 끄면 보조를 계속 허용합니다.",
    "0=keep steering assist with blinkers, 1=pause below lane-change speed.",
    "0=방향지시등 중에도 보조 유지, 1=차선 변경 속도 아래에서 일시 정지.",
  ),
  "CruiseOverMaxSpeed": doc(
    "controlsd uses this to synchronize cruise max speed when current speed is already above the set speed. Turning it on can prevent the target from staying below actual speed; turning it off keeps the previous set speed.",
    "controlsd가 현재 속도가 설정 속도보다 높을 때 cruise max speed를 현재 속도 쪽으로 맞추는 데 사용합니다. 켜면 목표 속도가 실제 속도보다 낮게 남는 상황을 줄이고, 끄면 이전 설정 속도를 유지합니다.",
    "0=keep set speed, 1=raise/reset to current speed when over.",
    "0=설정 속도 유지, 1=초과 시 현재 속도 쪽으로 올림/재설정.",
  ),
  "OSMEnable": doc(
    "Starts or enables map-data support used by OSM speed-limit and curve-deceleration paths. By itself it mainly makes map data available; speed changes happen when the related speed-limit or curve options are enabled.",
    "OSM speed-limit 및 curve-deceleration 경로가 쓰는 map-data support를 시작하거나 활성화합니다. 이 값만으로는 주로 지도 데이터를 사용할 수 있게 하며, 실제 속도 변화는 관련 speed-limit 또는 curve 옵션이 켜졌을 때 발생합니다.",
    "0=do not request OSM/map support, 1=allow map support.",
    "0=OSM/map support 요청 안 함, 1=map support 허용.",
  ),
  "OSMSpeedLimitEnable": doc(
    "navicontrol uses liveMapData speed limits to lower the cruise target when appropriate. Turning it on can make the car slow for mapped speed limits; turning it off ignores OSM speed-limit values.",
    "navicontrol이 liveMapData의 속도 제한을 사용해 필요 시 cruise 목표를 낮춥니다. 켜면 지도 속도 제한에 맞춰 감속할 수 있고, 끄면 OSM 속도 제한 값을 무시합니다.",
    "0=ignore OSM speed limits, 1=use OSM speed limits.",
    "0=OSM 속도 제한 무시, 1=OSM 속도 제한 사용.",
  ),
  "StockNaviSpeedEnabled": doc(
    "controlsd/navicontrol use stock navigation safety-camera speed and distance when the car publishes them. Turning it on can make cruise slow for stock safety-camera sections; turning it off ignores that stock source.",
    "차량이 순정 내비 안전카메라 속도와 거리를 제공할 때 controlsd/navicontrol이 사용합니다. 켜면 순정 안전카메라 구간에서 cruise가 감속할 수 있고, 끄면 이 순정 source를 무시합니다.",
    "0=ignore stock safety-camera speed, 1=use it.",
    "0=순정 안전카메라 속도 무시, 1=사용.",
  ),
  "OpkrSpeedLimitOffsetOption": doc(
    "Selects how speed-limit offset is applied. Percent or positive absolute offsets make the target faster than the posted limit; negative or lower targets make speed-camera and speed-limit driving more conservative.",
    "속도 제한 offset 적용 방식을 선택합니다. percent 또는 양수 절대 offset은 목표 속도를 제한 속도보다 빠르게 만들고, 음수 또는 낮은 target은 안전카메라/속도 제한 주행을 더 보수적으로 만듭니다.",
    "0=percent style, 1=absolute/custom style in this menu, 2=custom CSV table in code paths that support it.",
    "0=percent 방식, 1=이 메뉴의 절대/custom 방식, 2=지원 경로에서 custom CSV 표 사용.",
  ),
  "OpkrSpeedLimitOffset": doc(
    "Offset value used by speed-limit and Waze/OSM speed-control paths. Higher positive values allow a faster cruise target above the detected limit; lower or negative values slow earlier.",
    "speed-limit 및 Waze/OSM 속도 제어 경로가 사용하는 offset입니다. 양수 값을 키우면 감지 제한보다 빠른 cruise 목표를 허용하고, 낮거나 음수이면 더 일찍 느리게 갑니다.",
    "Unit depends on OpkrSpeedLimitOffsetOption: percent or speed-unit offset.",
    "단위는 OpkrSpeedLimitOffsetOption에 따라 percent 또는 속도 단위 offset입니다.",
  ),
  "OSMCustomSpeedLimitC": doc(
    "Input speed-limit breakpoints for the custom speed-limit table. When selected by the offset option, each detected limit is mapped to an OSMCustomSpeedLimitT target.",
    "custom speed-limit 표의 입력 속도 제한 breakpoint입니다. offset option에서 선택된 경우 감지된 각 제한 속도를 OSMCustomSpeedLimitT 목표 속도로 매핑합니다.",
    "Comma-separated speed limits. Count must match OSMCustomSpeedLimitT.",
    "comma로 구분한 속도 제한입니다. OSMCustomSpeedLimitT와 개수가 같아야 합니다.",
  ),
  "OSMCustomSpeedLimitT": doc(
    "Target cruise speeds paired with OSMCustomSpeedLimitC. Lower targets make the car slow more below the posted limit; higher targets allow faster travel through the same limit.",
    "OSMCustomSpeedLimitC와 짝을 이루는 목표 cruise 속도입니다. 낮은 target은 표시 제한보다 더 느리게 가게 하고, 높은 target은 같은 제한 구간에서 더 빠른 주행을 허용합니다.",
    "Comma-separated target speeds. Count must match OSMCustomSpeedLimitC.",
    "comma로 구분한 목표 속도입니다. OSMCustomSpeedLimitC와 개수가 같아야 합니다.",
  ),
  "OpkrSpeedLimitSignType": doc(
    "UI/display preference for safety-camera sign style. It does not directly change acceleration or braking; speed behavior is controlled by the safety-camera and speed-limit options.",
    "안전카메라 표지 모양에 대한 UI/display 설정입니다. 가속이나 제동을 직접 바꾸지는 않으며, 속도 동작은 safety-camera 및 speed-limit 옵션이 제어합니다.",
    "0=circle, 1=rectangle.",
    "0=원형, 1=사각형.",
  ),
  "SafetyCamDecelDistGain": doc(
    "navicontrol multiplies the safety-camera deceleration start distance by this gain. Higher values start slowing earlier and more gently; lower values delay braking and can feel more abrupt.",
    "navicontrol이 안전카메라 감속 시작 거리에 이 gain을 곱합니다. 값을 높이면 더 일찍 부드럽게 감속을 시작하고, 낮추면 제동이 늦어져 더 급하게 느껴질 수 있습니다.",
    "Percent gain. Effective distance multiplier is 1 + raw * 0.01.",
    "percent gain입니다. 실제 거리 배율은 1 + raw * 0.01입니다.",
  ),
  "CurvDecelOption": doc(
    "Selects which curve-speed source can lower the cruise target. Vision uses model predicted speed, OSM uses map turnSpeedLimit. More sources can slow for more curves; None disables this automatic curve slowdown.",
    "cruise 목표를 낮출 수 있는 커브 속도 source를 선택합니다. Vision은 모델 예측 속도, OSM은 지도 turnSpeedLimit을 사용합니다. source가 많을수록 더 많은 커브에서 감속할 수 있고, None은 자동 커브 감속을 끕니다.",
    "0=None, 1=Vision+OSM, 2=Vision only, 3=OSM only.",
    "0=None, 1=Vision+OSM, 2=Vision only, 3=OSM only.",
  ),
  "VCurvSpeedC": doc(
    "Vision curve-speed breakpoints from modelSpeed. They are paired with VCurvSpeedT; lower target values make vision curve deceleration stronger.",
    "modelSpeed 기반 vision curve-speed breakpoint입니다. VCurvSpeedT와 짝이며, target 값을 낮게 두면 vision 커브 감속이 강해집니다.",
    "Comma-separated curve speed inputs in metric mode. Count must match VCurvSpeedT.",
    "metric mode의 curve speed 입력을 comma로 구분합니다. VCurvSpeedT와 개수가 같아야 합니다.",
  ),
  "VCurvSpeedT": doc(
    "Target speeds for VCurvSpeedC. Lowering these targets slows the car more in model-detected curves; raising them keeps speed higher through curves.",
    "VCurvSpeedC에 대응하는 목표 속도입니다. 낮추면 모델이 감지한 커브에서 더 많이 감속하고, 높이면 커브 통과 속도를 더 유지합니다.",
    "Comma-separated target speeds. Count must match VCurvSpeedC.",
    "comma로 구분한 목표 속도입니다. VCurvSpeedC와 개수가 같아야 합니다.",
  ),
  "VCurvSpeedCMPH": doc(
    "Imperial-mode vision curve-speed breakpoints. They mirror VCurvSpeedC for mph-based setups.",
    "imperial mode용 vision curve-speed breakpoint입니다. mph 기반 설정에서 VCurvSpeedC와 같은 역할을 합니다.",
    "Comma-separated mph inputs. Count must match VCurvSpeedTMPH.",
    "comma로 구분한 mph 입력입니다. VCurvSpeedTMPH와 개수가 같아야 합니다.",
  ),
  "VCurvSpeedTMPH": doc(
    "Imperial-mode target speeds for vision curve deceleration. Lower targets slow more for model curves; higher targets preserve speed.",
    "imperial mode용 vision curve 감속 목표 속도입니다. 낮추면 모델 커브에서 더 감속하고, 높이면 속도를 더 유지합니다.",
    "Comma-separated mph targets. Count must match VCurvSpeedCMPH.",
    "comma로 구분한 mph 목표입니다. VCurvSpeedCMPH와 개수가 같아야 합니다.",
  ),
  "OCurvSpeedC": doc(
    "OSM turnSpeedLimit breakpoints for map-based curve deceleration. They map incoming turn-speed limits to target cruise speeds.",
    "지도 기반 커브 감속에서 쓰는 OSM turnSpeedLimit breakpoint입니다. 들어오는 turn-speed limit을 목표 cruise 속도로 매핑합니다.",
    "Comma-separated OSM turn-speed inputs. Count must match OCurvSpeedT.",
    "comma로 구분한 OSM turn-speed 입력입니다. OCurvSpeedT와 개수가 같아야 합니다.",
  ),
  "OCurvSpeedT": doc(
    "Target cruise speeds for OCurvSpeedC. Lower values make map curves slow more; higher values keep speed up when OSM curve data is present.",
    "OCurvSpeedC에 대응하는 목표 cruise 속도입니다. 낮추면 지도 커브에서 더 감속하고, 높이면 OSM 커브 데이터가 있어도 속도를 더 유지합니다.",
    "Comma-separated target speeds. Count must match OCurvSpeedC.",
    "comma로 구분한 목표 속도입니다. OCurvSpeedC와 개수가 같아야 합니다.",
  ),
  "OPKRSpeedBump": doc(
    "navicontrol uses speed-bump signs to lower target speed near a bump. Turning it on makes the car slow for detected bumps; turning it off ignores that sign type.",
    "navicontrol이 과속방지턱 표지를 사용해 방지턱 근처 목표 속도를 낮춥니다. 켜면 감지된 방지턱에서 감속하고, 끄면 이 표지를 무시합니다.",
    "0=ignore speed-bump signs, 1=slow for speed bumps.",
    "0=방지턱 표지 무시, 1=방지턱 감속.",
  ),
  "OPKREarlyStop": doc(
    "Hyundai controller can temporarily increase stock SCC gap when it sees early stop conditions from lead or model stopline. Turning it on starts slowing earlier; turning it off keeps normal gap behavior.",
    "Hyundai controller가 앞차 또는 모델 stopline에서 조기 정지 조건을 보면 순정 SCC gap을 임시로 키울 수 있습니다. 켜면 더 일찍 감속하고, 끄면 일반 gap 동작을 유지합니다.",
    "0=normal gap behavior, 1=early slowdown by temporary gap increase.",
    "0=일반 gap 동작, 1=임시 gap 증가로 조기 감속.",
  ),
  "AutoEnable": doc(
    "controlsd auto-engage gate. When enabled together with UFCModeEnabled, openpilot may engage automatically after the cruise standby conditions and speed threshold are satisfied. Turning it off requires normal manual engagement.",
    "controlsd auto-engage gate입니다. UFCModeEnabled와 함께 켜져 있으면 cruise standby 조건과 속도 임계값을 만족한 뒤 openpilot이 자동 engage될 수 있습니다. 끄면 일반 수동 engage가 필요합니다.",
    "0=manual engage only, 1=allow auto engage when other gates pass.",
    "0=수동 engage만 허용, 1=다른 gate 통과 시 auto engage 허용.",
  ),
  "AutoEnableSpeed": doc(
    "Minimum speed for the auto-engage path. Raising it delays automatic engagement until the car is faster; lowering it allows engagement earlier. Negative values are preserved by the code as special values.",
    "auto-engage 경로의 최소 속도입니다. 높이면 더 빠른 속도에서만 자동 engage되고, 낮추면 더 이른 속도에서 허용됩니다. 음수는 코드에서 특수값으로 보존됩니다.",
    "kph in this OPKR path.",
    "이 OPKR 경로에서는 km/h입니다.",
  ),
  "CruiseAutoRes": doc(
    "Hyundai controller can send RES or SET messages to re-enable cruise after cancellation when the configured conditions pass. Turning it on makes cruise recovery more automatic; turning it off leaves recovery to the driver.",
    "설정 조건을 통과하면 Hyundai controller가 cancel 이후 cruise를 다시 켜기 위해 RES 또는 SET 메시지를 보낼 수 있습니다. 켜면 cruise 복귀가 더 자동화되고, 끄면 운전자에게 맡깁니다.",
    "0=manual cruise recovery, 1=allow AutoRES.",
    "0=수동 cruise 복귀, 1=AutoRES 허용.",
  ),
  "AutoResOption": doc(
    "Selects which button action AutoRES uses. RES resumes the previous set speed, SET can set around current speed, and lead-aware mode chooses based on lead presence. This changes how aggressively cruise returns after cancel.",
    "AutoRES가 사용할 버튼 동작을 선택합니다. RES는 이전 설정 속도로 복귀하고, SET은 현재 속도 근처로 설정할 수 있으며, lead-aware mode는 앞차 유무에 따라 선택합니다. cancel 이후 cruise 복귀의 적극성이 달라집니다.",
    "Menu labels are compatibility labels; verify the exact behavior on the target car before road use.",
    "메뉴 label은 호환 label입니다. 실제 도로 사용 전 대상 차량에서 정확한 동작을 확인해야 합니다.",
  ),
  "AutoResCondition": doc(
    "Condition selector for AutoRES. The current Hyundai controller code mainly distinguishes the default path from gas-pressed gated paths, so non-default options should be treated as requiring driver intent.",
    "AutoRES 조건 선택값입니다. 현재 Hyundai controller 코드는 주로 기본 경로와 gas-pressed gate 경로를 구분하므로, non-default 옵션은 운전자 의도를 요구하는 설정으로 다뤄야 합니다.",
    "0=default/brake-release style, nonzero options gate more tightly in current code.",
    "0=기본/brake-release 계열, 0이 아닌 옵션은 현재 코드에서 더 강하게 gate됩니다.",
  ),
  "AutoResLimitTime": doc(
    "Maximum time window in which AutoRES is allowed after a cancel/brake event. Higher values keep the automatic resume opportunity open longer; lower values expire it sooner.",
    "cancel/brake 이후 AutoRES를 허용하는 최대 시간 창입니다. 높이면 자동 복귀 기회가 더 오래 유지되고, 낮추면 더 빨리 만료됩니다.",
    "Seconds.",
    "초 단위입니다.",
  ),
  "AutoRESDelay": doc(
    "Delay before AutoRES sends its cruise button action. Higher delay waits longer after conditions become valid; lower delay resumes sooner.",
    "AutoRES가 cruise 버튼 동작을 보내기 전 대기 시간입니다. 높이면 조건이 만족된 뒤 더 기다리고, 낮추면 더 빨리 복귀합니다.",
    "Seconds.",
    "초 단위입니다.",
  ),
  "LaneWidth": doc(
    "lane_planner initializes the lane-width estimate from this value. Wider values make the planner assume lanes are wider when confidence is low; narrower values pull the assumed lane center closer.",
    "lane_planner가 차선 폭 추정 초기값으로 사용합니다. 값을 넓게 두면 confidence가 낮을 때 더 넓은 차선을 가정하고, 좁게 두면 가정한 차선 중심이 더 가까워집니다.",
    "Raw value is multiplied by 0.1 meters.",
    "raw 값에 0.1 m를 곱합니다.",
  ),
  "SpdLaneWidthSpd": doc(
    "Speed breakpoints for speed-based lane-width fallback. They decide at which speeds SpdLaneWidthSet values are interpolated when lane width confidence is weak.",
    "속도 기반 차선 폭 fallback의 속도 breakpoint입니다. 차선 폭 confidence가 약할 때 어떤 속도에서 SpdLaneWidthSet 값을 보간할지 정합니다.",
    "Comma-separated speeds in m/s in lane_planner. Count must match SpdLaneWidthSet.",
    "lane_planner에서는 m/s 속도 목록입니다. SpdLaneWidthSet과 개수가 같아야 합니다.",
  ),
  "SpdLaneWidthSet": doc(
    "Lane-width values paired with SpdLaneWidthSpd. Larger widths shift the inferred lane center outward when fallback is used; smaller widths make the assumed lane narrower.",
    "SpdLaneWidthSpd와 짝을 이루는 차선 폭 값입니다. fallback이 사용될 때 큰 값은 추정 차선 중심을 바깥쪽으로 넓게 잡고, 작은 값은 차선을 좁게 가정합니다.",
    "Comma-separated meters. Count must match SpdLaneWidthSpd.",
    "comma로 구분한 m 값입니다. SpdLaneWidthSpd와 개수가 같아야 합니다.",
  ),
  "RoutineDriveOn": doc(
    "lane_planner can apply road-name based camera offsets from liveMapData when this is enabled and RoutineDriveOption includes the offset mode. Turning it on can shift lane centering on known roads.",
    "이 값이 켜져 있고 RoutineDriveOption이 offset mode를 포함하면 lane_planner가 liveMapData의 도로명 기반 camera offset을 적용할 수 있습니다. 켜면 알려진 도로에서 차선 중앙 위치가 이동할 수 있습니다.",
    "0=ignore routine road offsets, 1=allow routine-drive options.",
    "0=routine 도로 offset 무시, 1=routine-drive 옵션 허용.",
  ),
  "RoutineDriveOption": doc(
    "Character option list for routine-drive behavior. In lane_planner, option '0' enables roadCameraOffset; in navicontrol, option '1' enables road-name speed-limit matching.",
    "routine-drive 동작을 고르는 문자 option 목록입니다. lane_planner에서는 '0'이 roadCameraOffset을 켜고, navicontrol에서는 '1'이 도로명 speed-limit matching을 켭니다.",
    "Original OPKR text format. Use only documented option characters.",
    "기존 OPKR 문자열 형식입니다. 확인된 option 문자만 사용합니다.",
  ),
  "CloseToRoadEdge": doc(
    "lane_planner uses road-edge and outer-lane probabilities to add an edge offset. Turning it on can bias the car closer to the road edge on first/last lanes; turning it off keeps normal lane centering.",
    "lane_planner가 road-edge 및 바깥 차선 probability를 보고 edge offset을 더합니다. 켜면 1차로/끝차로에서 차량 위치가 도로 가장자리 쪽으로 치우칠 수 있고, 끄면 일반 차선 중앙을 유지합니다.",
    "0=no road-edge offset, 1=use LeftEdgeOffset/RightEdgeOffset when edge conditions match.",
    "0=road-edge offset 없음, 1=edge 조건이 맞을 때 LeftEdgeOffset/RightEdgeOffset 사용.",
  ),
  "LeftEdgeOffset": doc(
    "Offset used by CloseToRoadEdge when the left road-edge condition matches. Larger absolute values move the planned center more; use small changes because this directly shifts lateral position.",
    "CloseToRoadEdge에서 왼쪽 road-edge 조건이 맞을 때 쓰는 offset입니다. 절대값이 클수록 계획 중심을 더 많이 이동시키므로, 횡방향 위치를 직접 바꾸는 값이라 작게 조정해야 합니다.",
    "Raw value is multiplied by 0.01 meters.",
    "raw 값에 0.01 m를 곱합니다.",
  ),
  "RightEdgeOffset": doc(
    "Offset used by CloseToRoadEdge when the right road-edge condition matches. Larger absolute values move the planned center more; use small changes because this directly shifts lateral position.",
    "CloseToRoadEdge에서 오른쪽 road-edge 조건이 맞을 때 쓰는 offset입니다. 절대값이 클수록 계획 중심을 더 많이 이동시키므로, 횡방향 위치를 직접 바꾸는 값이라 작게 조정해야 합니다.",
    "Raw value is multiplied by 0.01 meters.",
    "raw 값에 0.01 m를 곱합니다.",
  ),
  "AvoidLKASFaultEnabled": doc(
    "Hyundai controller periodically cuts steering torque when steering angle stays above the configured angle/frame limit, to avoid stock LKAS temporary faults. Turning it on may cause brief assist dropouts in high-angle turns; turning it off keeps continuous assist but risks stock faults.",
    "Hyundai controller가 설정된 angle/frame 제한 이상으로 조향각이 유지되면 순정 LKAS temporary fault를 피하기 위해 조향 토크를 주기적으로 끊습니다. 켜면 큰 조향각에서 짧은 보조 끊김이 생길 수 있고, 끄면 보조는 계속되지만 순정 fault 위험이 커집니다.",
    "0=continuous steering limit path, 1=LKAS-fault avoidance cutout.",
    "0=연속 조향 제한 경로, 1=LKAS fault 회피 cutout.",
  ),
  "AvoidLKASFaultMaxAngle": doc(
    "Steering angle threshold for LKAS fault avoidance. Lower values start cutouts earlier; higher values allow more steering angle before cutout.",
    "LKAS fault 회피의 조향각 임계값입니다. 낮추면 cutout이 더 일찍 시작되고, 높이면 더 큰 조향각까지 허용합니다.",
    "Degrees.",
    "도 단위입니다.",
  ),
  "AvoidLKASFaultMaxFrame": doc(
    "Frame count before LKAS fault avoidance cuts torque after the angle threshold is exceeded. Higher values wait longer before cutout; lower values cut sooner.",
    "조향각 임계값을 넘은 뒤 LKAS fault 회피가 토크를 끊기 전 기다리는 frame 수입니다. 높이면 cutout 전 더 오래 기다리고, 낮추면 더 빨리 끊습니다.",
    "Controller frame count.",
    "controller frame 수입니다.",
  ),
  "SpeedCameraOffset": doc(
    "lane_planner adds a speed-dependent camera offset when enabled. It is strongest at low speed and fades to zero near highway speed, so turning it on can correct low-speed lateral bias without affecting high-speed driving much.",
    "켜져 있으면 lane_planner가 속도에 따른 camera offset을 더합니다. 저속에서 가장 강하고 고속에 가까워질수록 0으로 줄어, 고속 주행에는 크게 영향 없이 저속 횡방향 치우침을 보정할 수 있습니다.",
    "0=off, 1=apply about -0.10 m at 0 m/s, fading to 0 near 31 m/s.",
    "0=꺼짐, 1=0 m/s에서 약 -0.10 m 적용 후 31 m/s 근처에서 0으로 감소.",
  ),

  "CameraOffsetAdj": doc(
    "lane_planner shifts both detected lane lines before computing the lane center. Positive raw values become negative meters in this code, so they move the perceived lane center in the opposite sign direction. This directly changes where the car centers in the lane.",
    "lane_planner가 차선 중심을 계산하기 전에 검출된 양쪽 차선선을 이동시킵니다. 이 코드에서는 양수 raw가 음수 m로 바뀌므로 부호 반대 방향으로 차선 중심 인식이 이동합니다. 차량이 차선 안에서 어디를 중심으로 달릴지 직접 바꿉니다.",
    "Raw integer millimeters, converted to -(raw * 0.001) m on EON-style paths.",
    "원시 정수 mm이며 EON 계열 경로에서 -(raw * 0.001) m로 변환됩니다.",
  ),
  "PathOffsetAdj": doc(
    "lane_planner adds this offset to the model path y coordinate. Unlike CameraOffsetAdj, it shifts the planned path itself, so it directly moves the target trajectory left or right.",
    "lane_planner가 모델 path의 y 좌표에 이 offset을 더합니다. CameraOffsetAdj와 달리 계획 경로 자체를 이동하므로 목표 궤적을 좌우로 직접 옮깁니다.",
    "Raw integer millimeters, converted to -(raw * 0.001) m.",
    "원시 정수 mm이며 -(raw * 0.001) m로 변환됩니다.",
  ),
  "SteerActuatorDelayAdj": doc(
    "Hyundai CarInterface stores this as steerActuatorDelay. Higher delay tells the lateral planner the steering actuator reacts later, so control becomes more anticipatory but can feel late or overshoot if too high. Lower delay makes response assumptions quicker.",
    "Hyundai CarInterface가 steerActuatorDelay로 저장합니다. 값을 높이면 lateral planner가 조향 actuator가 늦게 반응한다고 보고 더 앞서 제어하지만, 너무 높으면 늦거나 overshoot처럼 느껴질 수 있습니다. 낮추면 더 빠르게 반응한다고 가정합니다.",
    "Raw value is multiplied by 0.01 seconds.",
    "raw 값에 0.01초를 곱합니다.",
  ),
  "TireStiffnessFactorAdj": doc(
    "Vehicle-model tire stiffness scale. Higher values make the model assume the car responds more strongly to steering; lower values assume softer tires. Wrong values can cause lane-centering bias, sluggishness, or oscillation.",
    "차량 모델의 타이어 강성 scale입니다. 값을 높이면 같은 조향에 차가 더 강하게 반응한다고 가정하고, 낮추면 타이어가 더 무르다고 가정합니다. 맞지 않으면 차선 중앙 오차, 둔한 반응, 진동이 생길 수 있습니다.",
    "Raw value is multiplied by 0.01.",
    "raw 값에 0.01을 곱합니다.",
  ),
  "SteerThreshold": doc(
    "Hyundai steering threshold used by car/controller constants. Higher values make the system less sensitive to small driver steering torque; lower values make driver steering override/interaction easier to detect.",
    "Hyundai car/controller 상수로 쓰이는 조향 threshold입니다. 값을 높이면 작은 운전자 조향 토크에 덜 민감하고, 낮추면 운전자 조향 override/개입을 더 쉽게 감지합니다.",
    "Raw Hyundai controller unit.",
    "Hyundai controller 원시 단위입니다.",
  ),
  "SteerLimitTimerAdj": doc(
    "Hyundai CarInterface stores this as steerLimitTimer. Higher values allow sustained steering limit longer before limit behavior; lower values trigger limiting sooner.",
    "Hyundai CarInterface가 steerLimitTimer로 저장합니다. 값을 높이면 조향 제한 상태를 더 오래 허용하고, 낮추면 제한 동작이 더 빨리 걸립니다.",
    "Raw value is multiplied by 0.01 seconds.",
    "raw 값에 0.01초를 곱합니다.",
  ),
  "OpkrLiveSteerRatio": doc(
    "controlsd can adjust steerRatio live between the base and max values. Turning it on lets steering ratio change with speed/angle, which can improve curve tracking but may change steering feel while driving.",
    "controlsd가 base와 max 사이에서 steerRatio를 live로 조정할 수 있게 합니다. 켜면 속도/조향각에 따라 steer ratio가 바뀌어 커브 추종이 좋아질 수 있지만 주행 중 조향 느낌도 달라질 수 있습니다.",
    "0=fixed steerRatio from CarParams, 1=live steer-ratio adjustment.",
    "0=CarParams steerRatio 고정, 1=live steer-ratio 조정.",
  ),
  "LiveSteerRatioPercent": doc(
    "Bias percentage for live steer-ratio adjustment. Higher positive values push the live ratio upward, generally reducing steering command for the same curvature; lower values push it downward and make steering more responsive.",
    "live steer-ratio 조정의 percent bias입니다. 양수로 높이면 live ratio가 커져 같은 곡률에서 조향 명령이 대체로 줄고, 낮추면 ratio가 작아져 조향 반응이 커집니다.",
    "Signed percent.",
    "부호 있는 percent입니다.",
  ),
  "SteerRatioAdj": doc(
    "Base steer ratio stored in CarParams. Higher steer ratio assumes more steering-wheel angle is needed for the same tire angle, which usually softens lateral response; lower values make response stronger.",
    "CarParams에 저장되는 base steer ratio입니다. 높이면 같은 타이어 각에 더 큰 핸들 각이 필요하다고 가정해 일반적으로 lateral response가 부드러워지고, 낮추면 반응이 강해집니다.",
    "Raw value is multiplied by 0.01. 1550 means 15.50.",
    "raw 값에 0.01을 곱합니다. 1550은 15.50입니다.",
  ),
  "SteerRatioMaxAdj": doc(
    "Upper steer-ratio bound for live steer ratio. Higher max lets live tuning soften steering more in the conditions that choose the upper bound; lower max keeps steering response closer to the base value.",
    "live steer ratio의 상한입니다. max를 높이면 상한을 선택하는 조건에서 조향을 더 부드럽게 만들 수 있고, 낮추면 base 값에 더 가깝게 유지됩니다.",
    "Raw value is multiplied by 0.01.",
    "raw 값에 0.01을 곱합니다.",
  ),
  "OpkrVariableSteerMax": doc(
    "Hyundai controller can interpolate maximum steering torque from modelSpeed. Turning it on allows stronger steering torque at lower modelSpeed/high-curvature cases and base torque at higher speed; turning it off always uses SteerMaxBaseAdj.",
    "Hyundai controller가 modelSpeed로 최대 조향 토크를 보간할 수 있게 합니다. 켜면 낮은 modelSpeed/큰 곡률 상황에서 더 강한 조향 토크를 허용하고 고속에서는 base 토크를 사용합니다. 끄면 항상 SteerMaxBaseAdj를 씁니다.",
    "0=SteerMaxBaseAdj only, 1=interpolate SteerMaxAdj to SteerMaxBaseAdj.",
    "0=SteerMaxBaseAdj만 사용, 1=SteerMaxAdj와 SteerMaxBaseAdj 사이 보간.",
  ),
  "SteerMaxBaseAdj": doc(
    "Base maximum steering torque used by Hyundai controller. Higher values allow stronger lane-centering and curve torque but can feel aggressive or approach stock limits; lower values soften steering and may understeer in curves.",
    "Hyundai controller가 사용하는 base 최대 조향 토크입니다. 값을 높이면 차선 중앙 유지와 커브 토크가 강해지지만 거칠거나 순정 제한에 가까워질 수 있고, 낮추면 부드럽지만 커브에서 부족할 수 있습니다.",
    "Raw Hyundai steering torque unit.",
    "Hyundai 조향 토크 원시 단위입니다.",
  ),
  "SteerMaxAdj": doc(
    "Maximum steering torque used by the variable steer-max path. Higher values give more steering authority in low modelSpeed/high-curvature cases; lower values reduce peak torque.",
    "variable steer-max 경로에서 쓰는 최대 조향 토크입니다. 값을 높이면 낮은 modelSpeed/큰 곡률 상황에서 조향 권한이 커지고, 낮추면 peak 토크가 줄어듭니다.",
    "Raw Hyundai steering torque unit.",
    "Hyundai 조향 토크 원시 단위입니다.",
  ),
  "OpkrVariableSteerDelta": doc(
    "Hyundai controller can interpolate steering torque rate limits. Turning it on allows different torque ramp speeds by modelSpeed; turning it off keeps base ramp limits.",
    "Hyundai controller가 조향 토크 변화율 제한을 보간할 수 있게 합니다. 켜면 modelSpeed에 따라 토크 ramp 속도가 달라지고, 끄면 base ramp 제한을 유지합니다.",
    "0=base delta limits, 1=variable delta limits.",
    "0=base delta 제한, 1=variable delta 제한.",
  ),
  "SteerDeltaUpBaseAdj": doc(
    "Base limit for how quickly requested steering torque may increase. Higher values let torque build faster and track sharp changes better; lower values smooth the response.",
    "요청 조향 토크가 증가할 수 있는 base 변화율 제한입니다. 값을 높이면 토크가 더 빨리 올라 급한 변화를 잘 따라가고, 낮추면 반응이 부드러워집니다.",
    "Raw Hyundai controller delta unit.",
    "Hyundai controller delta 원시 단위입니다.",
  ),
  "SteerDeltaUpAdj": doc(
    "Maximum increase-rate limit used by variable steer-delta. Higher values allow faster torque buildup in selected conditions; lower values keep steering calmer.",
    "variable steer-delta에서 쓰는 최대 증가율 제한입니다. 값을 높이면 선택 조건에서 토크가 더 빨리 올라가고, 낮추면 조향이 더 차분합니다.",
    "Raw Hyundai controller delta unit.",
    "Hyundai controller delta 원시 단위입니다.",
  ),
  "SteerDeltaDownBaseAdj": doc(
    "Base limit for how quickly requested steering torque may decrease. Higher values let torque unwind faster; lower values release torque more gradually.",
    "요청 조향 토크가 감소할 수 있는 base 변화율 제한입니다. 값을 높이면 토크가 더 빨리 풀리고, 낮추면 더 서서히 풀립니다.",
    "Raw Hyundai controller delta unit.",
    "Hyundai controller delta 원시 단위입니다.",
  ),
  "SteerDeltaDownAdj": doc(
    "Maximum decrease-rate limit used by variable steer-delta. Higher values let the controller remove torque faster after curves or corrections; lower values make release smoother.",
    "variable steer-delta에서 쓰는 최대 감소율 제한입니다. 값을 높이면 커브나 보정 후 토크를 더 빨리 제거하고, 낮추면 release가 더 부드럽습니다.",
    "Raw Hyundai controller delta unit.",
    "Hyundai controller delta 원시 단위입니다.",
  ),
  "AvoidLKASFaultBeyond": doc(
    "When LKAS fault avoidance is enabled and steering angle is already large at low speed, this allows the controller to use the stronger max steer/delta values. Turning it on can keep more assist in tight turns, but needs careful fault testing.",
    "LKAS fault avoidance가 켜져 있고 저속에서 이미 조향각이 큰 경우, controller가 더 강한 max steer/delta 값을 쓰도록 허용합니다. 켜면 타이트한 회전에서 보조를 더 유지할 수 있지만 fault 테스트가 필요합니다.",
    "0=use normal limits, 1=allow stronger beyond-angle limits in the guarded condition.",
    "0=일반 제한 사용, 1=guard 조건에서 더 강한 beyond-angle 제한 허용.",
  ),
  "DesiredCurvatureLimit": doc(
    "drive_helpers multiplies the desired-curvature rate limit by this value. Higher values let desired curvature change faster and can follow sharp geometry sooner; lower values damp sudden curvature changes but may lag in tight curves.",
    "drive_helpers가 desired-curvature 변화율 제한에 이 값을 곱합니다. 높이면 desired curvature가 더 빨리 바뀌어 급한 geometry를 더 빨리 따르고, 낮추면 갑작스런 곡률 변화를 누그러뜨리지만 타이트한 커브에서 늦을 수 있습니다.",
    "Raw value is multiplied by 0.01.",
    "raw 값에 0.01을 곱합니다.",
  ),
  "PidKp": doc(
    "PID lateral proportional gain. Higher Kp reacts more strongly to current lateral error and can reduce lane offset faster, but too high can oscillate. Lower Kp is calmer but may track wide in curves.",
    "PID lateral 비례 gain입니다. Kp를 높이면 현재 lateral error에 더 강하게 반응해 차선 오차를 빨리 줄일 수 있지만, 너무 높으면 진동할 수 있습니다. 낮추면 차분하지만 커브에서 넓게 돌 수 있습니다.",
    "Effective Kp = raw * 0.01.",
    "실제 Kp = raw * 0.01.",
  ),
  "PidKi": doc(
    "PID integral gain. Higher Ki removes steady lane-centering bias faster, but too high can build up and overshoot after long curves. Lower Ki leaves more persistent bias.",
    "PID 적분 gain입니다. Ki를 높이면 지속적인 차선 중앙 오차를 더 빨리 없애지만, 너무 높으면 긴 커브 뒤 누적되어 overshoot가 날 수 있습니다. 낮추면 지속 bias가 더 남습니다.",
    "Effective Ki = raw * 0.001.",
    "실제 Ki = raw * 0.001.",
  ),
  "PidKd": doc(
    "PID derivative gain. Higher Kd damps fast error changes and can reduce oscillation; too high can make steering noisy or resistant. Lower Kd gives less damping.",
    "PID 미분 gain입니다. Kd를 높이면 빠른 error 변화를 감쇠해 진동을 줄일 수 있지만, 너무 높으면 조향이 거칠거나 버티는 느낌이 날 수 있습니다. 낮추면 감쇠가 줄어듭니다.",
    "Effective Kd = raw * 0.01.",
    "실제 Kd = raw * 0.01.",
  ),
  "PidKf": doc(
    "PID feedforward gain. Higher Kf adds more steering from the predicted curvature before feedback error appears; lower Kf relies more on feedback correction.",
    "PID feedforward gain입니다. Kf를 높이면 feedback error가 생기기 전 예측 곡률에서 더 많은 조향을 넣고, 낮추면 feedback 보정에 더 의존합니다.",
    "Effective Kf = raw * 0.00001.",
    "실제 Kf = raw * 0.00001.",
  ),
  "InnerLoopGain": doc(
    "INDI inner-loop gain for steering-rate correction. Higher values make rate correction stronger and quicker; lower values smooth the inner loop but can lag.",
    "INDI steering-rate 보정용 inner-loop gain입니다. 높이면 rate 보정이 강하고 빨라지고, 낮추면 inner loop가 부드럽지만 늦을 수 있습니다.",
    "Effective value = raw * 0.1.",
    "실제 값 = raw * 0.1.",
  ),
  "OuterLoopGain": doc(
    "INDI outer-loop gain for angle/lateral error response. Higher values make lane-centering response stronger; lower values make it calmer but less eager.",
    "INDI angle/lateral error 응답용 outer-loop gain입니다. 높이면 차선 중앙 유지 반응이 강해지고, 낮추면 차분하지만 덜 적극적입니다.",
    "Effective value = raw * 0.1.",
    "실제 값 = raw * 0.1.",
  ),
  "TimeConstant": doc(
    "INDI actuator time constant. Higher values assume slower actuator dynamics and smooth changes more; lower values assume quicker steering response.",
    "INDI actuator time constant입니다. 높이면 actuator가 더 느리다고 가정해 변화를 더 완만하게 만들고, 낮추면 조향 반응이 더 빠르다고 가정합니다.",
    "Effective value = raw * 0.1.",
    "실제 값 = raw * 0.1.",
  ),
  "ActuatorEffectiveness": doc(
    "INDI actuator effectiveness. It changes how much steering effect the controller expects from a command. Higher values can reduce commanded effort for the same desired effect; lower values can make commands stronger.",
    "INDI actuator effectiveness입니다. controller가 명령 하나에서 기대하는 조향 효과를 바꿉니다. 높이면 같은 목표 효과에 필요한 명령이 줄 수 있고, 낮추면 명령이 더 강해질 수 있습니다.",
    "Effective value = raw * 0.1.",
    "실제 값 = raw * 0.1.",
  ),
  "Scale": doc(
    "LQR scale value. It changes the state scaling used by the LQR controller, so large changes can strongly alter steering response and stability.",
    "LQR scale 값입니다. LQR controller의 state scaling을 바꾸므로 큰 변경은 조향 반응과 안정성을 크게 바꿀 수 있습니다.",
    "Stored as raw numeric value.",
    "원시 숫자 값으로 저장됩니다.",
  ),
  "LqrKi": doc(
    "LQR integral gain. Higher Ki removes steady offset faster but can add overshoot; lower Ki is safer but leaves more bias.",
    "LQR 적분 gain입니다. 높이면 지속 offset을 빨리 제거하지만 overshoot가 생길 수 있고, 낮추면 안정적이지만 bias가 더 남습니다.",
    "Effective Ki = raw * 0.001.",
    "실제 Ki = raw * 0.001.",
  ),
  "DcGain": doc(
    "LQR dcGain for the lateral plant model. Changing it alters how steering command maps to lateral response; wrong values can make tracking weak or oscillatory.",
    "lateral plant model의 LQR dcGain입니다. 바꾸면 조향 명령이 lateral response로 매핑되는 방식이 달라지며, 맞지 않으면 추종이 약하거나 진동할 수 있습니다.",
    "Effective dcGain = raw * 0.00001.",
    "실제 dcGain = raw * 0.00001.",
  ),
  "TorqueMaxLatAccel": doc(
    "Torque controller maximum lateral acceleration scale. It normalizes TorqueKp/Kf/Ki. Higher values reduce the effective normalized gains for the same raw gains; lower values make them stronger.",
    "Torque controller의 최대 lateral acceleration scale입니다. TorqueKp/Kf/Ki를 정규화합니다. 값을 높이면 같은 raw gain의 실제 정규화 gain이 약해지고, 낮추면 더 강해집니다.",
    "Effective max_lat_accel = raw * 0.1 m/s^2.",
    "실제 max_lat_accel = raw * 0.1 m/s^2.",
  ),
  "TorqueKp": doc(
    "Torque controller proportional gain. Higher effective Kp commands stronger steering torque for lateral acceleration error, improving response but risking oscillation. Lower Kp is smoother but can track wide.",
    "Torque controller 비례 gain입니다. 실제 Kp를 높이면 lateral acceleration error에 더 강한 조향 토크를 명령해 반응이 좋아질 수 있지만 진동 위험이 있습니다. 낮추면 부드럽지만 넓게 추종할 수 있습니다.",
    "Effective Kp = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 Kp = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueKf": doc(
    "Torque controller feedforward gain. Higher effective Kf adds more predicted torque from desired curvature before feedback error grows; lower Kf relies more on feedback.",
    "Torque controller feedforward gain입니다. 실제 Kf를 높이면 feedback error가 커지기 전 목표 곡률에서 더 많은 예측 토크를 넣고, 낮추면 feedback에 더 의존합니다.",
    "Effective Kf = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 Kf = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueKi": doc(
    "Torque controller integral gain. Higher effective Ki removes steady torque bias faster; too high can build up and overshoot. Lower Ki leaves more residual bias.",
    "Torque controller 적분 gain입니다. 실제 Ki를 높이면 지속 토크 bias를 빨리 없애지만 너무 높으면 누적되어 overshoot가 날 수 있습니다. 낮추면 남는 bias가 커집니다.",
    "Effective Ki = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
    "실제 Ki = raw * 0.1 / (TorqueMaxLatAccel * 0.1).",
  ),
  "TorqueFriction": doc(
    "Torque controller friction compensation. Higher values overcome steering friction/deadband more aggressively; too high can cause small steering chatter. Lower values may leave sluggish center response.",
    "Torque controller friction 보상입니다. 높이면 조향 마찰/deadband를 더 적극적으로 넘지만 너무 높으면 작은 조향 떨림이 생길 수 있습니다. 낮추면 중앙 부근 반응이 둔할 수 있습니다.",
    "Effective friction = raw * 0.001.",
    "실제 friction = raw * 0.001.",
  ),
  "TorqueUseAngle": doc(
    "Torque controller option to include steering-angle feedback. Turning it on can improve angle-based correction on supported tunes; turning it off keeps the pure torque-style path.",
    "Torque controller에서 steering-angle feedback을 포함할지 선택합니다. 켜면 지원 튜닝에서 angle 기반 보정이 좋아질 수 있고, 끄면 순수 torque 계열 경로를 유지합니다.",
    "0=do not use steering angle, 1=use steering angle.",
    "0=steering angle 미사용, 1=steering angle 사용.",
  ),
  "TorqueAngDeadZone": doc(
    "Deadzone for steering-angle feedback in torque control. Higher values ignore more small angle error and reduce twitchiness; lower values react to smaller angle differences.",
    "torque control의 steering-angle feedback deadzone입니다. 높이면 작은 angle error를 더 무시해 민감한 움직임이 줄고, 낮추면 더 작은 각도 차이에도 반응합니다.",
    "Raw value is multiplied by 0.1 degrees.",
    "raw 값에 0.1도를 곱합니다.",
  ),
  "CustomTREnabled": doc(
    "long MPC uses custom following-time gap values when enabled. Turning it on makes lead following distance follow CruiseGap/DynamicTR settings; turning it off returns to the fixed 1.45 s time gap.",
    "켜져 있으면 long MPC가 custom 추종 시간 간격을 사용합니다. 켜면 앞차 추종 거리가 CruiseGap/DynamicTR 설정을 따르고, 끄면 고정 1.45초 time gap으로 돌아갑니다.",
    "0=fixed 1.45 s TR, 1=custom TR.",
    "0=고정 1.45초 TR, 1=custom TR.",
  ),
  "CruiseGap1": doc(
    "Desired following time gap for stock cruise gap level 1. Higher values leave more distance and brake earlier behind a lead; lower values follow closer.",
    "순정 cruise gap 1단계의 목표 추종 시간 간격입니다. 높이면 앞차 뒤에서 거리를 더 남기고 더 일찍 감속하며, 낮추면 더 가깝게 따라갑니다.",
    "Raw value is multiplied by 0.1 seconds.",
    "raw 값에 0.1초를 곱합니다.",
  ),
  "CruiseGap2": doc(
    "Desired following time gap for stock cruise gap level 2. Higher values are more conservative; lower values make following closer.",
    "순정 cruise gap 2단계의 목표 추종 시간 간격입니다. 높이면 더 보수적이고, 낮추면 앞차를 더 가깝게 따라갑니다.",
    "Raw value is multiplied by 0.1 seconds.",
    "raw 값에 0.1초를 곱합니다.",
  ),
  "CruiseGap3": doc(
    "Desired following time gap for stock cruise gap level 3. Higher values increase lead distance; lower values shorten it.",
    "순정 cruise gap 3단계의 목표 추종 시간 간격입니다. 높이면 앞차 거리가 늘고, 낮추면 짧아집니다.",
    "Raw value is multiplied by 0.1 seconds.",
    "raw 값에 0.1초를 곱합니다.",
  ),
  "CruiseGap4": doc(
    "Desired following time gap for stock cruise gap level 4. Higher values make the longest gap more conservative; lower values reduce the maximum following distance.",
    "순정 cruise gap 4단계의 목표 추종 시간 간격입니다. 높이면 가장 긴 gap이 더 보수적으로 동작하고, 낮추면 최대 추종 거리가 줄어듭니다.",
    "Raw value is multiplied by 0.1 seconds.",
    "raw 값에 0.1초를 곱합니다.",
  ),
  "DynamicTRGap": doc(
    "Selects which cruise-gap level uses DynamicTR interpolation. Enabling it makes following distance vary with speed for that gap level; leaving it off keeps static CruiseGap values.",
    "어떤 cruise-gap 단계에 DynamicTR 보간을 적용할지 선택합니다. 켜면 해당 gap 단계의 추종 거리가 속도에 따라 달라지고, 끄면 고정 CruiseGap 값을 유지합니다.",
    "0=off, 1-4=replace that gap level with DynamicTR.",
    "0=꺼짐, 1-4=해당 gap 단계에 DynamicTR 적용.",
  ),
  "DynamicTRSpd": doc(
    "Speed breakpoints for dynamic following time. Use higher TR targets at higher speeds for more conservative highway following, or lower targets for closer following.",
    "dynamic following time의 속도 breakpoint입니다. 고속에서 더 큰 TR target을 두면 고속 추종이 보수적이고, 낮은 target을 두면 더 가깝게 따라갑니다.",
    "Comma-separated speeds in kph or mph according to IsMetric. Count must match DynamicTRSet.",
    "IsMetric에 따른 km/h 또는 mph 속도를 comma로 구분합니다. DynamicTRSet과 개수가 같아야 합니다.",
  ),
  "DynamicTRSet": doc(
    "Following-time targets interpolated over DynamicTRSpd. Higher values increase lead distance and earlier braking; lower values reduce following distance.",
    "DynamicTRSpd 위에서 보간되는 추종 시간 target입니다. 높이면 앞차 거리와 조기 제동이 늘고, 낮추면 추종 거리가 줄어듭니다.",
    "Comma-separated seconds. Count must match DynamicTRSpd.",
    "comma로 구분한 초 단위 값입니다. DynamicTRSpd와 개수가 같아야 합니다.",
  ),
  "RadarLongHelper": doc(
    "Hyundai longitudinal source selector. Vision Only relies on openpilot vision, Radar Only relies more on stock radar, and OPKR blends custom radar/vision behavior. Changing this can strongly alter lead following and braking feel.",
    "Hyundai longitudinal source 선택값입니다. Vision Only는 openpilot vision에, Radar Only는 순정 radar에 더 의존하고, OPKR은 custom radar/vision 동작을 섞습니다. 이 값은 앞차 추종과 제동 느낌을 크게 바꿀 수 있습니다.",
    "0=Vision Only, 1=Radar Only, 2=OPKR.",
    "0=Vision Only, 1=Radar Only, 2=OPKR.",
  ),
  "StoppingDistAdj": doc(
    "Hyundai controller adds extra deceleration near a lead when stopping-distance adjustment is enabled. Turning it on can stop farther back or more decisively near a lead; turning it off uses the normal long-control request.",
    "stopping-distance 보정이 켜져 있으면 Hyundai controller가 앞차 근처에서 추가 감속을 더합니다. 켜면 앞차 근처에서 더 뒤에 또는 더 단호하게 설 수 있고, 끄면 일반 long-control 요청을 사용합니다.",
    "0=normal stop request, 1=apply StoppingDist adjustment.",
    "0=일반 정지 요청, 1=StoppingDist 보정 적용.",
  ),
  "StoppingDist": doc(
    "Target distance used by StoppingDistAdj. Higher values begin extra deceleration farther from the lead; lower values allow stopping closer.",
    "StoppingDistAdj가 사용하는 목표 거리입니다. 높이면 앞차에서 더 먼 거리부터 추가 감속하고, 낮추면 더 가깝게 정지하도록 허용합니다.",
    "Raw value is multiplied by 0.1 meters.",
    "raw 값에 0.1 m를 곱합니다.",
  ),
  "E2ELong": doc(
    "long MPC switches toward end-to-end longitudinal policy when enabled. Turning it on makes speed planning depend more on model-predicted scene behavior; turning it off keeps the lead/radar-style policy.",
    "켜면 long MPC가 end-to-end longitudinal 정책 쪽으로 전환합니다. 켜면 속도 계획이 모델이 예측한 장면 동작에 더 의존하고, 끄면 lead/radar 계열 정책을 유지합니다.",
    "0=lead/radar-style longitudinal policy, 1=end-to-end longitudinal policy.",
    "0=lead/radar 계열 longitudinal 정책, 1=end-to-end longitudinal 정책.",
  ),
  "StopAtStopSign": doc(
    "Hyundai controller can use model stop-sign/stop-line style longitudinal plan when there is no lead. Turning it on can stop for model-detected stop conditions; turning it off ignores that stop-source path.",
    "앞차가 없을 때 Hyundai controller가 모델 stop-sign/stop-line 계열 longitudinal plan을 사용할 수 있게 합니다. 켜면 모델이 감지한 정지 조건에서 멈출 수 있고, 끄면 이 stop source를 무시합니다.",
    "0=ignore model stop-sign path, 1=allow it.",
    "0=model stop-sign 경로 무시, 1=허용.",
  ),
  "UseStockDecelOnSS": doc(
    "Hyundai controller can clamp requested acceleration down to the stock safety-section deceleration when it is stronger. Turning it on makes stock safety-camera decel more authoritative.",
    "순정 안전구간 감속 요청이 더 강할 때 Hyundai controller가 요청 가속도를 그 값으로 낮출 수 있습니다. 켜면 순정 안전카메라 감속이 더 우선됩니다.",
    "0=use openpilot decel request, 1=honor stronger stock safety-section decel.",
    "0=openpilot 감속 요청 사용, 1=더 강한 순정 안전구간 감속 우선.",
  ),
  "RadarDisable": doc(
    "Hyundai radar-disable path changes CarParams longitudinal mode and sends radar/SCC workaround messages. Turning it on can enable openpilot longitudinal on supported cars, but it is safety-critical and can change lead detection and braking behavior.",
    "Hyundai radar-disable 경로가 CarParams longitudinal mode를 바꾸고 radar/SCC workaround 메시지를 보냅니다. 켜면 지원 차량에서 openpilot longitudinal을 가능하게 할 수 있지만, 안전 중요 값이며 앞차 감지와 제동 동작을 바꿉니다.",
    "0=radar enabled/stock path, 1=radar-disable workaround path.",
    "0=radar 활성/순정 경로, 1=radar-disable workaround 경로.",
  ),
  "MultipleLateralUse": doc(
    "LatControlATOM mode selector. It decides whether lateral control is fixed or selected by speed/steering-angle interpolation, changing which controller actually steers the car.",
    "LatControlATOM mode 선택값입니다. lateral control을 고정할지, 속도/조향각 보간으로 고를지 정하며 실제로 차를 조향하는 controller가 달라집니다.",
    "0=PID, 1=INDI, 2=LQR/angle interpolation path, 3=TORQUE/speed interpolation path, 4=MULTI depending on ATOM code.",
    "0=PID, 1=INDI, 2=LQR/angle 보간 경로, 3=TORQUE/speed 보간 경로, 4=ATOM 코드 기준 MULTI.",
  ),
  "MultipleLateralSpd": doc(
    "Speed breakpoints for speed-based multi-lateral selection. They determine where ATOM changes or blends lateral controllers as speed rises.",
    "속도 기반 multi-lateral 선택의 speed breakpoint입니다. 속도가 올라갈 때 ATOM이 lateral controller를 바꾸거나 섞는 지점을 정합니다.",
    "Comma-separated speeds. Count must match MultipleLateralOpS.",
    "comma로 구분한 속도입니다. MultipleLateralOpS와 개수가 같아야 합니다.",
  ),
  "MultipleLateralOpS": doc(
    "Controller method ids paired with MultipleLateralSpd. Choosing stronger or weaker controllers at a speed band changes steering feel in that band.",
    "MultipleLateralSpd와 짝을 이루는 controller method id입니다. 특정 속도 구간에 강하거나 약한 controller를 고르면 그 구간의 조향 느낌이 달라집니다.",
    "Method ids: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
    "method id: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
  ),
  "MultipleLateralAng": doc(
    "Steering-angle breakpoints for angle-based multi-lateral selection. They determine where ATOM changes controller behavior as steering angle grows.",
    "조향각 기반 multi-lateral 선택의 angle breakpoint입니다. 조향각이 커질 때 ATOM이 controller 동작을 바꾸는 지점을 정합니다.",
    "Comma-separated steering angles. Count must match MultipleLateralOpA.",
    "comma로 구분한 조향각입니다. MultipleLateralOpA와 개수가 같아야 합니다.",
  ),
  "MultipleLateralOpA": doc(
    "Controller method ids paired with MultipleLateralAng. This changes which lateral controller handles small versus large steering-angle situations.",
    "MultipleLateralAng와 짝을 이루는 controller method id입니다. 작은 조향각과 큰 조향각 상황을 어떤 lateral controller가 맡을지 바꿉니다.",
    "Method ids: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
    "method id: 0=PID, 1=INDI, 2=LQR, 3=TORQUE.",
  ),
  "MaxSteer": doc(
    "Developer Hyundai maximum steering torque constant. Raising it allows stronger steering authority but increases fault/comfort risk; lowering it softens or limits steering.",
    "Developer용 Hyundai 최대 조향 토크 상수입니다. 높이면 조향 권한이 커지지만 fault/승차감 위험이 늘고, 낮추면 조향이 부드럽거나 제한됩니다.",
    "Raw Hyundai steering torque unit.",
    "Hyundai 조향 토크 원시 단위입니다.",
  ),
  "MaxRTDelta": doc(
    "Developer real-time steering delta limit. Higher values allow larger short-term torque changes; lower values make torque changes smoother but can reduce curve authority.",
    "Developer용 real-time 조향 delta 제한입니다. 높이면 짧은 시간의 토크 변화를 더 크게 허용하고, 낮추면 변화가 부드럽지만 커브 조향 권한이 줄 수 있습니다.",
    "Raw Hyundai controller delta unit.",
    "Hyundai controller delta 원시 단위입니다.",
  ),
  "MaxRateUp": doc(
    "Developer maximum steering rate-up constant. Higher values build steering torque faster; lower values smooth torque buildup.",
    "Developer용 최대 조향 rate-up 상수입니다. 높이면 조향 토크가 더 빨리 올라가고, 낮추면 토크 상승이 부드러워집니다.",
    "Raw Hyundai controller rate unit.",
    "Hyundai controller rate 원시 단위입니다.",
  ),
  "MaxRateDown": doc(
    "Developer maximum steering rate-down constant. Higher values release steering torque faster; lower values release it more gradually.",
    "Developer용 최대 조향 rate-down 상수입니다. 높이면 조향 토크가 더 빨리 풀리고, 낮추면 더 서서히 풀립니다.",
    "Raw Hyundai controller rate unit.",
    "Hyundai controller rate 원시 단위입니다.",
  ),
}

SETTING_DOCS.update(DRIVING_EFFECT_DOCS)

FAMILY_DOCS: List[tuple[set[str], HelpDoc]] = [
  ({"MaxSteer", "MaxRTDelta", "MaxRateUp", "MaxRateDown", "SteerMaxBaseAdj", "SteerMaxAdj",
    "SteerDeltaUpBaseAdj", "SteerDeltaUpAdj", "SteerDeltaDownBaseAdj", "SteerDeltaDownAdj",
    "SteerThreshold"},
   doc("Hyundai steering-limit value used by car/controller tuning code to bound requested steering torque and rate.",
       "Hyundai 조향 제한 코드가 요청 조향 토크와 변화율을 제한할 때 사용하는 값입니다.",
       "Raw integer in the same unit as the Hyundai controller constants.",
       "Hyundai controller 상수와 같은 단위의 원시 정수입니다.")),
  ({"LaneWidth", "SpdLaneWidthSpd", "SpdLaneWidthSet"},
   doc("Read by lane_planner to estimate lane width and optionally interpolate a target lane width by speed.",
       "lane_planner가 차선 폭을 추정하고 필요 시 속도별 목표 차선 폭을 보간할 때 읽습니다.",
       "LaneWidth raw is multiplied by 0.1 m; speed/value lists must have matching counts.",
       "LaneWidth raw는 0.1 m를 곱합니다. 속도/값 목록은 개수가 맞아야 합니다.")),
  ({"LeftCurvOffsetAdj", "RightCurvOffsetAdj", "SpeedCameraOffset", "CloseToRoadEdge", "LeftEdgeOffset", "RightEdgeOffset"},
   doc("Read by lane_planner to add curve, speed, and road-edge dependent offsets to the detected lane-line center.",
       "lane_planner가 곡률, 속도, 도로 가장자리 조건에 따른 offset을 검출 차선 중심에 더할 때 읽습니다.",
       "Edge offsets use raw * 0.01 m where applicable.",
       "edge offset 계열은 적용 시 raw * 0.01 m를 사용합니다.")),
  ({"CruiseGapBySpdOn", "CruiseGapBySpdSpd", "CruiseGapBySpdGap", "CruiseGapAdjust",
    "StandstillResumeAlt", "DepartChimeAtResume", "OpkrAutoResume", "RESCountatStandstill",
    "OpkrVariableCruise", "VarCruiseSpeedFactor", "CruiseOverMaxSpeed", "CruiseAutoRes",
    "AutoResOption", "AutoResCondition", "AutoResLimitTime", "AutoRESDelay"},
   doc("OPKR cruise/resume behavior setting read by controlsd or Hyundai state/controller code to alter resume button, cruise gap, or set-speed behavior.",
       "controlsd 또는 Hyundai state/controller 코드가 resume 버튼, cruise gap, set speed 동작을 바꿀 때 읽는 OPKR cruise/resume 설정입니다.",
       "Most toggles are 0/1; list settings must keep matching item counts with their paired list.",
       "대부분 0/1 토글이며, 목록형 설정은 짝이 되는 목록과 개수가 맞아야 합니다.")),
  ({"CruiseSetwithRoadLimitSpeedEnabled", "CruiseSetwithRoadLimitSpeedOffset", "CruiseStatemodeSelInit",
    "OSMEnable", "OSMSpeedLimitEnable", "StockNaviSpeedEnabled", "OpkrSpeedLimitOffsetOption",
    "OpkrSpeedLimitOffset", "OSMCustomSpeedLimitC", "OSMCustomSpeedLimitT", "OpkrSpeedLimitSignType",
    "SafetyCamDecelDistGain", "CurvDecelOption", "VCurvSpeedC", "VCurvSpeedT", "VCurvSpeedCMPH",
    "VCurvSpeedTMPH", "OCurvSpeedC", "OCurvSpeedT", "OPKRSpeedBump", "OPKREarlyStop",
    "SpeedCameraOffset"},
   doc("OPKR speed-limit, safety-camera, and curve-deceleration setting used by planning/control helper paths when those map or stock-navigation inputs are available.",
       "지도 또는 순정 내비 입력이 있을 때 planning/control 보조 경로가 사용하는 OPKR 속도제한, 안전카메라, 커브 감속 설정입니다.",
       "Paired CSV breakpoints and targets must have matching counts.",
       "짝이 되는 CSV breakpoint/target 목록은 개수가 같아야 합니다.")),
  ({"OpkrLaneChangeSpeed", "OpkrAutoLaneChangeDelay", "LCTimingFactorEnable", "LCTimingFactorUD",
    "LCTimingFactor30", "LCTimingFactor60", "LCTimingFactor80", "LCTimingFactor110",
    "OpkrBlindSpotDetect", "OpkrTurnSteeringDisable"},
   doc("OPKR lane-change setting read by planning/control code to gate lane changes by speed, delay, blinkers, and blind-spot state.",
       "planning/control 코드가 속도, 지연, 방향지시등, blind-spot 상태로 차선변경을 제한할 때 읽는 OPKR 차선변경 설정입니다.",
       "Speeds are in the unit selected by IsMetric unless a code path explicitly converts them.",
       "코드가 별도 변환하지 않는 한 속도 단위는 IsMetric 선택을 따릅니다.")),
  ({"AutoEnable", "AutoEnableSpeed"},
   doc("Read by controlsd auto-enable logic. When conditions are safe and calibration is ready, openpilot can auto-engage above the configured speed.",
       "controlsd auto-enable 로직이 읽습니다. 조건이 안전하고 calibration이 준비되면 설정 속도 이상에서 openpilot이 자동 engage될 수 있습니다.",
       "Speed is kph in this OPKR path.",
       "이 OPKR 경로에서는 km/h 속도값입니다.")),
  ({"RoutineDriveOn", "RoutineDriveOption"},
   doc("Read by lane_planner for OPKR road-name based offset behavior through liveMapData when that map data is available.",
       "lane_planner가 liveMapData를 통해 도로명 기반 offset 동작을 적용할 때 읽는 OPKR 설정입니다.",
       "Text option follows the original OPKR route-option format.",
       "문자열 옵션은 기존 OPKR route-option 형식을 따릅니다.")),
  ({"AvoidLKASFaultEnabled", "AvoidLKASFaultMaxAngle", "AvoidLKASFaultMaxFrame", "AvoidLKASFaultBeyond",
    "DesiredCurvatureLimit"},
   doc("OPKR steering-safety setting used by controller paths to avoid LKAS faults or limit desired curvature/steering requests.",
       "controller 경로가 LKAS fault를 피하거나 desired curvature/steering 요청을 제한할 때 사용하는 OPKR 조향 안전 설정입니다.",
       "Raw values follow the controller-side Hyundai steering units.",
       "원시 값은 controller 쪽 Hyundai 조향 단위를 따릅니다.")),
  ({"OpkrMaxAngleLimit", "OpkrMaxSteeringAngle", "OpkrMaxDriverAngleWait", "OpkrMaxSteerAngleWait",
    "OpkrDriverAngleWait", "OpkrSteerAngleCorrection", "OpkrSteerMethod", "StockLKASEnabled",
    "UFCModeEnabled", "FCA11Message", "WhitePandaSupport", "NoSmartMDPS", "LdwsCarFix",
    "JustDoGearD", "SteerWarningFix", "IgnoreCANErroronISG", "ComIssueGone", "FingerprintTwoSet",
    "UserSpecificFeature", "JoystickDebugMode", "C2WithCommaPower", "OpkrBattLess"},
   doc("Hyundai/OPKR vehicle-integration compatibility setting. These are read by carstate, carcontroller, board, or safety workaround paths rather than the model pipeline.",
       "Hyundai/OPKR 차량 통합 호환 설정입니다. 모델 파이프라인이 아니라 carstate, carcontroller, board, safety workaround 경로에서 읽습니다.",
       "Most are 0/1 toggles; numeric waits are seconds in the existing controller code.",
       "대부분 0/1 토글이며, wait 계열 숫자는 기존 controller 코드에서 초 단위로 쓰입니다.")),
  ({"OpkrDrivingRecord", "RecordingCount", "RecordingQuality", "AnimatedRPM", "AnimatedRPMMax",
    "OpkrEnableGetoffAlert", "HoldForSetting", "RTShield", "ShowError", "PutPrebuiltOn"},
   doc("Original OPKR UI/system setting retained from the Qt menu. It is editable here for compatibility; some K230 headless runs may not start the original component that reads it.",
       "기존 Qt 메뉴에서 가져온 OPKR UI/system 설정입니다. 호환을 위해 편집 가능하지만, K230 headless 실행에서는 이 값을 읽는 기존 구성요소가 시작되지 않을 수 있습니다.",
       "Use the original default unless the matching OPKR component is running.",
       "해당 OPKR 구성요소가 실행 중일 때만 기본값에서 변경하는 것이 좋습니다.")),
  ({"StoppingDistAdj", "StopAtStopSign", "UseStockDecelOnSS", "RadarDisable"},
   doc("OPKR longitudinal stopping/radar behavior setting used by long-control and planning variants when those features are active.",
       "해당 기능이 활성일 때 long-control 및 planning 변형 경로가 사용하는 OPKR longitudinal 정지/레이더 설정입니다.",
       "Treat changes as drive-control tuning changes.",
       "주행 제어 튜닝 변경으로 취급해야 합니다.")),
]

WEB_MESSAGE_SOURCES = {
  "app_title": "K230 Settings",
  "subtitle": "openpilot Params editor",
  "search_placeholder": "Search settings",
  "on": "On",
  "off": "Off",
  "saved": "Saved",
  "save_failed": "Save failed",
  "load_failed": "Load failed",
  "done": "Done",
  "running": "Running",
  "confirm": "Are you sure?",
  "type_to_confirm": "Type %1 to confirm",
  "input_required": "Input is required",
  "no_settings": "No settings",
  "debug": "Debug",
  "default": "Default",
  "changed": "Changed",
  "matches_default": "Default OK",
  "live": "Live",
  "value_guide": "Value / unit",
  "details": "Function",
  "no_debug_data": "No debug data",
  "updated": "Updated",
  "stale": "Stale",
}

WEB_KO_TRANSLATIONS = {
  "K230 Settings": "K230 설정",
  "openpilot Params editor": "openpilot 설정 편집기",
  "Search settings": "설정 검색",
  "On": "켜짐",
  "Off": "꺼짐",
  "Saved": "저장됨",
  "Save failed": "저장 실패",
  "Load failed": "불러오기 실패",
  "Done": "완료",
  "Running": "실행 중",
  "Are you sure?": "계속할까요?",
  "Type %1 to confirm": "확인을 위해 %1 를 입력하세요",
  "Input is required": "입력이 필요합니다",
  "No settings": "설정 없음",
  "Debug": "디버그",
  "Default": "기본값",
  "Changed": "변경됨",
  "Default OK": "기본값",
  "Live": "라이브",
  "Value / unit": "값 / 단위",
  "Function": "기능",
  "No debug data": "디버그 데이터 없음",
  "Updated": "갱신됨",
  "Stale": "오래됨",
  "Apply K7 HEV Defaults": "K7 HEV 기본값 적용",
  "Set the project default K7 Hybrid car profile and restore core steering/live tuning defaults.": "K7 하이브리드 차량 프로필과 주요 조향/라이브 튜닝 기본값을 적용합니다.",
}

_KO_TRANSLATION_CACHE: Optional[Dict[str, str]] = None


def normalize_language(language: Optional[str]) -> str:
  return language if language in SUPPORTED_LANGUAGES else LANG_EN


def load_korean_translations() -> Dict[str, str]:
  global _KO_TRANSLATION_CACHE
  if _KO_TRANSLATION_CACHE is not None:
    return _KO_TRANSLATION_CACHE
  translations = dict(WEB_KO_TRANSLATIONS)
  if KO_TRANSLATION_FILE.exists():
    try:
      root = ET.parse(KO_TRANSLATION_FILE).getroot()
      for msg in root.findall(".//message"):
        source = msg.findtext("source") or ""
        node = msg.find("translation")
        translation = (node.text or "").strip() if node is not None else ""
        if source and translation and node is not None and node.attrib.get("type") != "unfinished":
          translations.setdefault(source, translation)
    except ET.ParseError:
      pass
  _KO_TRANSLATION_CACHE = translations
  return translations


def translate_text(language: str, text: str) -> str:
  if not text or normalize_language(language) != LANG_KO:
    return text
  return load_korean_translations().get(text, text)


def messages_for(language: str) -> Dict[str, str]:
  return {key: translate_text(language, source) for key, source in WEB_MESSAGE_SOURCES.items()}


def current_language(params: Optional[Any] = None) -> str:
  try:
    params = params or new_params()
    return normalize_language(_decode_param(params.get("LanguageSetting")) or DEFAULT_PARAMS.get("LanguageSetting"))
  except Exception:
    return normalize_language(DEFAULT_PARAMS.get("LanguageSetting"))


def _read_list_file(paths: Iterable[Path]) -> List[str]:
  for path in paths:
    if path.exists():
      return [line.strip() for line in path.read_text(errors="ignore").splitlines() if line.strip()]
  return []


def _hyundai_car_models() -> List[str]:
  path = Path(BASEDIR) / "selfdrive/car/hyundai/values.py"
  if not path.exists():
    return []
  match = re.search(r"^class CAR:\n(.*?)(?:^\S|\Z)", path.read_text(errors="ignore"), re.S | re.M)
  if not match:
    return []
  return re.findall(r'^\s+[A-Z0-9_]+\s*=\s*"([^"]+)"', match.group(1), re.M)


def dynamic_options(source: Optional[str], params: Optional[Any] = None) -> List[Option]:
  if source == "timezone":
    values = _read_list_file([
      Path(BASEDIR) / "selfdrive/assets/addon/param/TimeZone",
      Path("/data/openpilot/selfdrive/assets/addon/param/TimeZone"),
    ])
    if "UTC" not in values:
      values.insert(0, "UTC")
    return [Option(value, value) for value in values]
  if source == "car_model":
    values = _read_list_file([
      Path.home() / ".comma/params/d/CarList",
      Path("/data/params/d/CarList"),
    ]) or _hyundai_car_models()
    try:
      current = _decode_param((params or new_params()).get("CarModel"))
      if current and current not in values:
        values.insert(0, current)
    except Exception:
      pass
    return [Option(value, value) for value in values]
  return []


def setting_default(setting: Setting) -> Optional[str]:
  if setting.type == "action" or setting.key.startswith("_"):
    return None
  return DEFAULT_PARAMS.get(setting.key, setting.default)


def _fallback_doc(setting: Setting) -> HelpDoc:
  if setting.type == "action":
    return doc(
      setting.description or "WebUI action implemented in selfdrive.webui.settings.",
      translate_text(LANG_KO, setting.description) or "selfdrive.webui.settings에 구현된 WebUI 액션입니다.",
      "Confirmation text is required when shown.",
      "확인 문구가 표시되면 그대로 입력해야 실행됩니다.",
    )
  if setting.readonly:
    return doc(
      "Read-only runtime or software value. WebUI displays the value but does not write it back to Params.",
      "읽기 전용 런타임 또는 소프트웨어 값입니다. WebUI는 표시만 하고 Params에 다시 쓰지 않습니다.",
    )
  details_en = (
    "This key is registered in params.cc and existed in the original Qt settings menu. "
    "No active reader was found in the K230 camerad/modeld/webui path inspected for this page, "
    "so it is kept here for compatibility with OPKR/openpilot components that may be started later."
  )
  details_ko = (
    "이 key는 params.cc에 등록되어 있고 기존 Qt 설정 메뉴에도 있던 항목입니다. "
    "이 페이지를 만들며 확인한 K230 camerad/modeld/webui 경로에서는 활성 reader를 찾지 못했으므로, "
    "나중에 실행될 수 있는 OPKR/openpilot 구성요소와의 호환을 위해 유지합니다."
  )
  if setting.type == "bool":
    return doc(details_en, details_ko, "0=off, 1=on.", "0=꺼짐, 1=켜짐.")
  if setting.type == "enum":
    return doc(details_en, details_ko, "The selected option value is stored as the raw Param string.", "선택한 option value가 원시 Param 문자열로 저장됩니다.")
  if setting.type == "number":
    return doc(details_en, details_ko, "Raw numeric Param. Keep the project default unless the matching component is active.", "원시 숫자 Param입니다. 해당 구성요소가 활성 상태가 아니라면 프로젝트 기본값을 유지합니다.")
  if setting.type == "csv":
    pair = f" Item count must match {setting.pair_key}." if setting.pair_key else ""
    pair_ko = f" 항목 개수는 {setting.pair_key}와 같아야 합니다." if setting.pair_key else ""
    return doc(details_en, details_ko, f"Comma-separated raw values.{pair}", f"comma로 구분한 원시 값입니다.{pair_ko}")
  return doc(details_en, details_ko, "Stored exactly as entered.", "입력한 문자열이 그대로 저장됩니다.")


def _setting_doc(setting: Setting) -> HelpDoc:
  if setting.key in SETTING_DOCS:
    return SETTING_DOCS[setting.key]
  for keys, help_doc in FAMILY_DOCS:
    if setting.key in keys:
      return help_doc
  return _fallback_doc(setting)


def setting_help(setting: Setting, language: str) -> tuple[str, str]:
  help_doc = _setting_doc(setting)
  if normalize_language(language) == LANG_KO:
    return help_doc.ko, help_doc.value_ko
  return help_doc.en, help_doc.value_en


def schema(language: Optional[str] = None, params: Optional[Any] = None) -> Dict[str, Any]:
  language = normalize_language(language or current_language(params))
  tr = lambda text: translate_text(language, text)
  settings = []
  for setting in SETTINGS:
    item = setting.to_dict(tr)
    item["default"] = setting_default(setting)
    if setting.source:
      item["options"] = [option.to_dict() for option in dynamic_options(setting.source, params)]
    details, value_guide = setting_help(setting, language)
    item["details"] = details
    item["value_guide"] = value_guide
    item["live"] = setting.key in LIVE_TUNE_KEYS
    settings.append(item)
  return {
    "panels": PANELS,
    "panel_titles": {panel: tr(panel) for panel in PANELS},
    "language": language,
    "messages": messages_for(language),
    "settings": settings,
    "actions": sorted(ACTIONS),
    "recommended_car_model": K7_HEV_MODEL,
  }


def _decode_param(value: Optional[bytes]) -> Optional[str]:
  if value is None:
    return None
  return value.decode("utf8", errors="replace")


def _ip_address() -> str:
  try:
    out = subprocess.check_output(["hostname", "-I"], text=True, timeout=1).strip()
    if out:
      return out.split()[0]
  except Exception:
    pass
  try:
    return socket.gethostbyname(socket.gethostname())
  except Exception:
    return ""


def new_params() -> Any:
  from common.params import Params
  return Params()


def read_value(params: Any, setting: Setting) -> Optional[str]:
  if setting.key == "_ip_address":
    return _ip_address()
  if setting.type == "action":
    return None
  if setting.type == "bool":
    return "1" if params.get_bool(setting.key) else "0"
  value = _decode_param(params.get(setting.key)) or DEFAULT_PARAMS.get(setting.key) or setting.default
  return normalize_language(value) if setting.key == "LanguageSetting" else value


def read_all(params: Optional[Any] = None) -> Dict[str, Optional[str]]:
  params = params or new_params()
  return {setting.key: read_value(params, setting) for setting in SETTINGS}


def _as_bool(value: Any) -> bool:
  if isinstance(value, bool):
    return value
  return str(value).lower() in ("1", "true", "yes", "on")


def write_value(key: str, value: Any, params: Optional[Any] = None) -> Optional[str]:
  params = params or new_params()
  setting = SETTINGS_BY_KEY.get(key)
  if setting is None:
    raise KeyError(key)
  if setting.readonly:
    raise PermissionError(key)
  validate_value(setting, value, params)
  if setting.type == "bool":
    params.put_bool(key, _as_bool(value))
  else:
    if key == "LanguageSetting":
      value = normalize_language(str(value))
    params.put(key, "" if value is None else str(value))
  return read_value(params, setting)


def _csv_items(value: Any) -> List[str]:
  return [item.strip() for item in str(value or "").split(",") if item.strip()]


def validate_value(setting: Setting, value: Any, params: Any) -> None:
  if setting.type == "csv" and setting.pair_key:
    pair = SETTINGS_BY_KEY[setting.pair_key]
    pair_value = read_value(params, pair)
    if len(_csv_items(value)) != len(_csv_items(pair_value)):
      raise ValueError(f"{setting.key} count must match {setting.pair_key}")


def _action_result(action_name: str, message: str, status: str = "ok",
                   output: str = "", exit_code: int = 0) -> Dict[str, Any]:
  result: Dict[str, Any] = {
    "status": status,
    "action": action_name,
    "message": message,
    "exit_code": exit_code,
  }
  if output:
    result["output"] = output[-4000:]
  return result


def _run_command(args: List[str], timeout: int = 60) -> subprocess.CompletedProcess:
  return subprocess.run(args, cwd=BASEDIR, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, timeout=timeout)


def _git_output(*args: str, timeout: int = 30) -> str:
  proc = _run_command(["git", *args], timeout=timeout)
  if proc.returncode != 0:
    raise RuntimeError(proc.stdout.strip() or f"git {' '.join(args)} failed")
  return proc.stdout.strip()


def _stamp_update(params: Any) -> None:
  params.put("LastUpdateTime", datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"))


def refresh(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
            dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("refresh", "dry run")
  params = params or new_params()
  params.put_bool("OnRoadRefresh", True)

  def clear_refresh() -> None:
    time.sleep(3)
    new_params().put_bool("OnRoadRefresh", False)

  threading.Thread(target=clear_refresh, daemon=True).start()
  return _action_result("refresh", "OnRoadRefresh pulsed")


def reset_calibration(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
                      dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("reset_calibration", "dry run")
  params = params or new_params()
  params.delete("CalibrationParams")
  params.delete("LiveParameters")
  refresh(params)
  return _action_result("reset_calibration", "CalibrationParams and LiveParameters removed")


def reboot(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
           dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("reboot", "dry run")
  (params or new_params()).put_bool("DoReboot", True)
  return _action_result("reboot", "DoReboot set")


def shutdown(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
             dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("shutdown", "dry run")
  (params or new_params()).put_bool("DoShutdown", True)
  return _action_result("shutdown", "DoShutdown set")


def check_update(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
                 dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("check_update", "dry run")
  params = params or new_params()
  try:
    branch = _git_output("rev-parse", "--abbrev-ref", "HEAD")
    local_hash = _git_output("rev-parse", "HEAD")
    fetch = _run_command(["git", "fetch", "origin"], timeout=90)
    if fetch.returncode != 0:
      _stamp_update(params)
      return _action_result("check_update", "git fetch failed", "fail", fetch.stdout, fetch.returncode)
    remote_hash = _git_output("rev-parse", "--verify", f"origin/{branch}")
    params.put("GitCommitRemote", remote_hash)
    _stamp_update(params)
    if local_hash == remote_hash:
      return _action_result("check_update", "Local and remote match. No update required.", output=fetch.stdout)
    return _action_result("check_update", "Remote update available.", output=f"LOCAL: {local_hash}\nREMOTE: {remote_hash}\n{fetch.stdout}")
  except Exception as exc:
    return _action_result("check_update", str(exc), "fail", exit_code=1)


def git_pull(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
             dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("git_pull", "dry run")
  params = params or new_params()
  try:
    branch = _git_output("rev-parse", "--abbrev-ref", "HEAD")
    proc = _run_command(["git", "pull", "--ff-only", "origin", branch], timeout=180)
    _stamp_update(params)
    if proc.returncode == 0:
      params.put("GitBranch", branch)
      params.put("GitCommit", _git_output("rev-parse", "HEAD"))
      params.put("GitRemote", _git_output("config", "--get", "remote.origin.url"))
      return _action_result("git_pull", "git pull completed", output=proc.stdout)
    return _action_result("git_pull", "git pull failed", "fail", proc.stdout, proc.returncode)
  except Exception as exc:
    return _action_result("git_pull", str(exc), "fail", exit_code=1)


def fetch_github_ssh_keys(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
                          dry_run: bool = False) -> Dict[str, Any]:
  payload = payload or {}
  username = str(payload.get("username") or "").strip()
  if not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?", username):
    raise ValueError("invalid GitHub username")
  if dry_run:
    return _action_result("fetch_github_ssh_keys", "dry run")
  try:
    with urllib.request.urlopen(f"https://github.com/{username}.keys", timeout=10) as response:
      keys = response.read().decode("utf8", errors="replace").strip()
  except urllib.error.URLError as exc:
    return _action_result("fetch_github_ssh_keys", f"GitHub request failed: {exc}", "fail", exit_code=1)
  if not keys:
    return _action_result("fetch_github_ssh_keys", f"Username '{username}' has no keys on GitHub", "fail", exit_code=1)
  params = params or new_params()
  params.put("GithubUsername", username)
  params.put("GithubSshKeys", keys)
  return _action_result("fetch_github_ssh_keys", "GitHub SSH keys saved", output=keys)


def remove_github_ssh_keys(params: Optional[Any] = None, payload: Optional[Dict[str, Any]] = None,
                           dry_run: bool = False) -> Dict[str, Any]:
  if dry_run:
    return _action_result("remove_github_ssh_keys", "dry run")
  params = params or new_params()
  params.delete("GithubUsername")
  params.delete("GithubSshKeys")
  params.put("OpkrSSHLegacy", "0")
  return _action_result("remove_github_ssh_keys", "GitHub SSH keys removed")


ACTIONS: Dict[str, Callable[..., Dict[str, Any]]] = {
  "reset_calibration": reset_calibration,
  "refresh": refresh,
  "reboot": reboot,
  "shutdown": shutdown,
  "check_update": check_update,
  "git_pull": git_pull,
  "fetch_github_ssh_keys": fetch_github_ssh_keys,
  "remove_github_ssh_keys": remove_github_ssh_keys,
}
_ACTION_LOCKS = {name: threading.Lock() for name in ACTIONS}


def run_action(name: str, payload: Optional[Dict[str, Any]] = None, params: Optional[Any] = None,
               dry_run: bool = False) -> Dict[str, Any]:
  action_fn = ACTIONS.get(name)
  if action_fn is None:
    raise KeyError(name)
  lock = _ACTION_LOCKS[name]
  if not lock.acquire(blocking=False):
    return _action_result(name, "action already running", "busy", exit_code=1)
  try:
    return action_fn(params=params, payload=payload, dry_run=dry_run)
  finally:
    lock.release()


def validate_registry(known_keys: Optional[Iterable[str]] = None) -> List[str]:
  if known_keys is None:
    params_cc = Path(BASEDIR) / "selfdrive/common/params.cc"
    known_keys = re.findall(r'\{"([^"]+)",', params_cc.read_text(errors="ignore"))
  known = set(known_keys)
  return sorted(key for key, setting in SETTINGS_BY_KEY.items() if not setting.readonly and key not in known)
