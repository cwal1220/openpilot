#!/usr/bin/env python3
import os
import math

from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from common.conversions import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET, CAMERA_OFFSET_A
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.longcontrol import LongControl
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.latcontrol_atom import LatControlATOM
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE, TICI, EON
from selfdrive.manager.process_config import managed_processes
from selfdrive.car.hyundai.values import Buttons
from decimal import Decimal

import common.log as trace1

SOFT_DISABLE_TIME = 3  # seconds
IS_KPH = Params().get_bool("IsMetric")
LDW_MIN_SPEED = 50 * CV.KPH_TO_MS if IS_KPH else 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
K230 = os.getenv("OPENPILOT_TARGET_ARCH") == "riscv64" or os.uname().machine == "riscv64"
K230_LATERAL_ONLY = os.getenv("K230_LATERAL_ONLY") == "1"
BLOCKED_PROCESSES = {p for p in os.getenv("BLOCK", "").split(",") if p}
if os.getenv("NOBOARD") is not None:
  BLOCKED_PROCESSES.add("pandad")
IGNORE_PROCESSES = {"rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned",
                    "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad", "statsd", "shutdownd"} | \
                    {k for k, v in managed_processes.items() if not v.enabled} | BLOCKED_PROCESSES

ACTUATOR_NUMERIC_FIELDS = ("gas", "brake", "steer", "steeringAngleDeg", "speed", "accel", "oaccel")
CLEAR_EVENT_TYPES = {
  (False, False): frozenset((ET.WARNING,)),
  (False, True): frozenset((ET.WARNING, ET.NO_ENTRY)),
  (True, False): frozenset(),
  (True, True): frozenset((ET.NO_ENTRY,)),
}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonEvent = car.CarState.ButtonEvent
SafetyModel = car.CarParams.SafetyModel
GearShifter = car.CarState.GearShifter

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None):
    config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)

    # Setup sockets
    self._publish_car_control = not (K230 and K230_LATERAL_ONLY)
    self.pm = pm
    if self.pm is None:
      pm_services = ['sendcan', 'controlsState', 'carState', 'carEvents', 'carParams']
      if self._publish_car_control:
        pm_services.append('carControl')
      self.pm = messaging.PubMaster(pm_services)

    self.has_driver_monitoring = not K230
    self.camera_packets = ["roadCameraState"] if K230 else ["roadCameraState", "driverCameraState"]
    if TICI:
      self.camera_packets.append("wideRoadCameraState")

    params = Params()
    self.params = params
    self.joystick_mode = params.get_bool("JoystickDebugMode")
    joystick_packet = ['testJoystick'] if self.joystick_mode else []

    self.sm = sm
    if self.sm is None:
      ignore = ['driverCameraState', 'managerState'] if SIMULATION else None
      services = ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                  'lateralPlan', 'liveLocationKalman', 'managerState', 'liveParameters']
      if not K230_LATERAL_ONLY:
        services.append('longitudinalPlan')
      if not K230_LATERAL_ONLY:
        services.append('radarState')
      if self.has_driver_monitoring:
        services.append('driverMonitoringState')
      self.sm = messaging.SubMaster(services + self.camera_packets + joystick_packet,
                                     ignore_alive=ignore, ignore_avg_freq=['radarState', 'longitudinalPlan'])

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    if TICI:
      self.log_sock = messaging.sub_sock('androidLog')

    # wait for one pandaState and one CAN packet
    print("Waiting for CAN messages...")
    get_one_can(self.can_sock)

    self.CI, self.CP, candidate = get_car(self.can_sock, self.pm.sock['sendcan'])
    if K230_LATERAL_ONLY:
      self.CP.openpilotLongitudinalControl = False
      if self.CI.CC is not None and hasattr(self.CI.CC, "longcontrol"):
        self.CI.CC.longcontrol = False
    self.CP.alternativeExperience = 0  # see panda/board/safety_declarations.h for allowed values
    self._cs_has_controls_v_cruise = hasattr(self.CI.CS, "controls_v_cruise")

    # read params
    self.is_metric = params.get_bool("IsMetric")
    self.is_ldw_enabled = params.get_bool("IsLdwEnabled")
    openpilot_enabled_toggle = params.get_bool("OpenpilotEnabledToggle")
    passive = params.get_bool("Passive") or not openpilot_enabled_toggle
    self.commIssue_ignored = params.get_bool("ComIssueGone")
    self.auto_enabled = params.get_bool("AutoEnable") and params.get_bool("UFCModeEnabled")
    self.batt_less = params.get_bool("OpkrBattLess")
    self.variable_cruise = params.get_bool('OpkrVariableCruise')
    self.cruise_over_maxspeed = params.get_bool('CruiseOverMaxSpeed')
    self.stock_lkas_on_disengaged_status = params.get_bool('StockLKASEnabled')
    self.no_mdps_mods = params.get_bool('NoSmartMDPS')

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    self.read_only = not car_recognized or not controller_available or self.CP.dashcamOnly
    if self.read_only:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write CarParams for radard
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)

    self.CC = car.CarControl.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP, candidate)
    self.VM = VehicleModel(self.CP)
    self._vm_stiffness = 1.0
    self._vm_steer_ratio = self.CP.steerRatio

    self.lateral_control_method = 0
    self.lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
      self.lateral_control_method = 5
    elif self.lat_tuning == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
      self.lateral_control_method = 0
    elif self.lat_tuning == 'indi':
      self.LaC = LatControlINDI(self.CP, self.CI)
      self.lateral_control_method = 1
    elif self.lat_tuning == 'lqr':
      self.LaC = LatControlLQR(self.CP, self.CI)
      self.lateral_control_method = 2
    elif self.lat_tuning == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)
      self.lateral_control_method = 3
    elif self.lat_tuning == 'atom':
      self.LaC = LatControlATOM(self.CP, self.CI)
      self.lateral_control_method = 4


    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self._carstate_controls_v_cruise = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.can_rcv_error_counter = 0
    self.last_blinker_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self._manager_process_frame = -1
    self._manager_process_not_running = False
    self._car_events_interval = int(1. / DT_CTRL)
    self._car_params_interval = int(50. / DT_CTRL)
    self.logged_comm_issue = False
    self.button_timers = {ButtonEvent.Type.decelCruise: 0, ButtonEvent.Type.accelCruise: 0}
    self.last_actuators = car.CarControl.Actuators.new_message()

    # TODO: no longer necessary, aside from process replay
    self.sm['liveParameters'].valid = True

    self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      #if len(self.CP.carFw) > 0:
      #  set_offroad_alert("Offroad_CarUnrecognized", True)
      #else:
      #  set_offroad_alert("Offroad_NoFirmware", True)
    elif self.read_only:
      self.events.add(EventName.dashcamMode, static=True)
    elif self.joystick_mode:
      self.events.add(EventName.joystickDebug, static=True)
      self.startup_event = None

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default

    self.hkg_stock_lkas = True
    self.hkg_stock_lkas_timer = 0

    self.mpc_frame = 0
    self.mpc_frame_sr = 0

    self.steerRatio_Max = float(Decimal(params.get("SteerRatioMaxAdj", encoding="utf8")) * Decimal('0.01'))
    self.steer_angle_range = [5, 30]
    self.steerRatio_range = [self.CP.steerRatio, self.steerRatio_Max]
    self.new_steerRatio = self.CP.steerRatio
    self.new_steerRatio_prev = self.CP.steerRatio
    self.steerRatio_to_send = 0
    self.live_sr = params.get_bool("OpkrLiveSteerRatio")
    self.live_sr_percent = int(params.get("LiveSteerRatioPercent", encoding="utf8"))

    self.steer_max = int(params.get("SteerMaxAdj", encoding="utf8"))

    self.second = 0.0
    self.lane_change_delay = int(params.get("OpkrAutoLaneChangeDelay", encoding="utf8"))
    auto_enable_speed = int(params.get("AutoEnableSpeed", encoding="utf8"))
    self.auto_enable_speed = max(1, auto_enable_speed) if auto_enable_speed > -1 else auto_enable_speed
    self.e2e_long_alert_prev = True
    self.unsleep_mode_alert_prev = True
    self.donotdisturb_mode_alert_prev = True
    self.ignore_can_error_on_isg = params.get_bool("IgnoreCANErroronISG")
    self.ready_timer = 0
    self.pause_spdlimit = False
    self.v_cruise_kph_set_timer = 0
    self.safety_speed = 0
    self.lkas_temporary_off = False
    self.gap_by_spd_on_temp = True

    self.var_cruise_speed_factor = int(params.get("VarCruiseSpeedFactor", encoding="utf8"))
    self.desired_angle_deg = 0

  def auto_enable(self, CS):
    if self.state != State.enabled:
      if CS.cruiseState.available and CS.vEgo >= self.auto_enable_speed * CV.KPH_TO_MS and CS.gearShifter == GearShifter.drive and \
       self.sm['liveCalibration'].calStatus != Calibration.UNCALIBRATED and self.initialized and self.ready_timer > 300:
        self.events.add( EventName.pcmEnable )

  def update_events(self, CS):
    """Compute carEvents from carState"""

    sm = self.sm
    self.events.clear()

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return

    self.events.add_from_msg(CS.events)
    if self.has_driver_monitoring:
      self.events.add_from_msg(sm['driverMonitoringState'].events)

    device_state = sm['deviceState']
    peripheral_state = sm['peripheralState']

    # Create events for battery, temperature, disk space, and memory
    if EON and (peripheral_state.pandaType != PandaType.uno) and \
       device_state.batteryPercent < 1 and device_state.chargingError and not self.batt_less:
      # at zero percent battery, while discharging, OP should not allowed
      self.events.add(EventName.lowBattery)
    if device_state.thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if device_state.freeSpacePercent < 7 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    # TODO: make tici threshold the same
    if device_state.memoryUsagePercent > (90 if TICI else 65) and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # TODO: enable this once loggerd CPU usage is more reasonable
    #cpus = list(self.sm['deviceState'].cpuUsagePercent)[:(-1 if EON else None)]
    #if max(cpus, default=0) > 95 and not SIMULATION:
    #  self.events.add(EventName.highCpuUsage)

    # Alert if fan isn't spinning for 5 seconds
    if peripheral_state.pandaType in (PandaType.uno, PandaType.dos):
      if peripheral_state.fanSpeedRpm == 0 and device_state.fanSpeedPercentDesired > 50:
        if (sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = sm.frame

    # Handle calibration status
    cal_status = sm['liveCalibration'].calStatus
    if cal_status != Calibration.CALIBRATED:
      if cal_status == Calibration.UNCALIBRATED:
        self.events.add(EventName.calibrationIncomplete)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    lateral_plan = sm['lateralPlan']
    if not self.lkas_temporary_off:
      if lateral_plan.laneChangeState == LaneChangeState.preLaneChange:
        direction = lateral_plan.laneChangeDirection
        if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
          (CS.rightBlindspot and direction == LaneChangeDirection.right):
          self.events.add(EventName.laneChangeBlocked)
        else:
          if direction == LaneChangeDirection.left:
            if self.lane_change_delay == 0:
              self.events.add(EventName.preLaneChangeLeft)
            else:
              self.events.add(EventName.laneChange)
          else:
            if self.lane_change_delay == 0:
              self.events.add(EventName.preLaneChangeRight)
            else:
              self.events.add(EventName.laneChange)
      elif lateral_plan.laneChangeState in (LaneChangeState.laneChangeStarting,
                                            LaneChangeState.laneChangeFinishing):
        self.events.add(EventName.laneChange)

    if self.can_rcv_error or (not CS.canValid and (not self.ignore_can_error_on_isg or CS.vEgo > 1)):
      self.events.add(EventName.canError)

    panda_states = sm['pandaStates']
    safety_configs = self.CP.safetyConfigs
    safety_configs_len = len(safety_configs)
    for i, pandaState in enumerate(panda_states):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < safety_configs_len:
        safety_config = safety_configs[i]
        safety_mismatch = pandaState.safetyModel != safety_config.safetyModel or \
                          pandaState.safetyParam != safety_config.safetyParam
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      if safety_mismatch or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)

    self.second += DT_CTRL
    if self.second > 1.0:
      params = self.params
      self.live_sr = params.get_bool("OpkrLiveSteerRatio")
      self.live_sr_percent = int(params.get("LiveSteerRatioPercent", encoding="utf8"))
      # E2ELongAlert
      e2e_long = params.get_bool("E2ELong")
      if e2e_long and self.e2e_long_alert_prev:
        self.events.add(EventName.e2eLongAlert)
        self.e2e_long_alert_prev = not self.e2e_long_alert_prev
      elif not e2e_long:
        self.e2e_long_alert_prev = True
      # UnSleep Mode Alert
      monitoring_mode = params.get_bool("OpkrMonitoringMode")
      if monitoring_mode and self.unsleep_mode_alert_prev:
        self.events.add(EventName.unSleepMode)
        self.unsleep_mode_alert_prev = not self.unsleep_mode_alert_prev
      elif not monitoring_mode:
        self.unsleep_mode_alert_prev = True
      # DoNotDisturb Mode Alert
      comma_stock_ui = params.get("CommaStockUI", encoding="utf8")
      if comma_stock_ui == "2" and self.donotdisturb_mode_alert_prev:
        self.events.add(EventName.doNotDisturb)
        self.donotdisturb_mode_alert_prev = not self.donotdisturb_mode_alert_prev
      elif comma_stock_ui != "2":
        self.donotdisturb_mode_alert_prev = True
      self.second = 0.0

    # if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
    #   self.events.add(EventName.relayMalfunction)

    # Check for HW or system issues
    sm_all_checks = sm.all_checks()
    if not K230_LATERAL_ONLY and len(sm['radarState'].radarErrors):
      self.events.add(EventName.radarFault)
    elif not sm.valid["pandaStates"]:
      self.events.add(EventName.usbError)
    elif not sm_all_checks or self.can_rcv_error:
      sm_all_alive = sm.all_alive()
      if self.commIssue_ignored:
        pass
      elif not sm_all_alive:
        self.events.add(EventName.commIssue)
      elif not sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else: # invalid or can_rcv_error.
        self.events.add(EventName.commIssue)

      if not self.logged_comm_issue:
        self.logged_comm_issue = True        
        service_list = sm.alive.keys()
        for s in service_list:
          if s not in sm.ignore_alive:
            print('{} = alive={} freq_ok={} valid={}'.format( s, sm.alive[s], sm.freq_ok[s], sm.valid[s] ) )
    else:
      self.logged_comm_issue = False

    #elif not self.sm.all_alive_and_valid() and not self.commIssue_ignored:
    #  self.events.add(EventName.commIssue)
    #  if not self.logged_comm_issue:
    #    invalid = [s for s, valid in self.sm.valid.items() if not valid]
    #    not_alive = [s for s, alive in self.sm.alive.items() if not alive]
    #    cloudlog.event("commIssue", invalid=invalid, not_alive=not_alive, can_error=self.can_rcv_error, error=True)
    #    self.logged_comm_issue = True
    #else:
    #  self.logged_comm_issue = False

    live_location = sm['liveLocationKalman']
    if not sm['liveParameters'].valid:
      self.events.add(EventName.vehicleModelInvalid)
    if not lateral_plan.mpcSolutionValid:
      self.events.add(EventName.plannerError)
    if not live_location.sensorsOK and not NOSENSOR:
      if sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
        self.events.add(EventName.sensorDataInvalid)
    if not live_location.posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not live_location.deviceStable:
      self.events.add(EventName.deviceFalling)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(3. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = (not K230_LATERAL_ONLY) and sm['longitudinalPlan'].fcw and self.enabled
    if planner_fcw or model_fcw:
      self.events.add(EventName.fcw)

    if TICI:
      logs = messaging.drain_sock(self.log_sock, wait_for_one=False)
      messages = []
      for m in logs:
        try:
          messages.append(m.androidLog.message)
        except UnicodeDecodeError:
          pass

      for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED"):
        for m in messages:
          if err not in m:
            continue

          csid = m.split("CSID:")[-1].split(" ")[0]
          evt = {"0": EventName.roadCameraError, "1": EventName.wideRoadCameraError,
                 "2": EventName.driverCameraError}.get(csid, None)
          if evt is not None:
            self.events.add(evt)

    # TODO: fix simulator
    if not SIMULATION:
      #if not NOSENSOR:
      #  if not self.sm['liveLocationKalman'].gpsOK and (self.distance_traveled > 1000):
      #    # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
      #    self.events.add(EventName.noGps)
      if CS.vEgo > 0.3 and not sm.all_alive(self.camera_packets):
        self.events.add(EventName.cameraMalfunction)
      if sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)
      if live_location.excessiveResets:
        self.events.add(EventName.localizerMalfunction)

      # Check if all manager processes are running
      manager_state_frame = sm.rcv_frame['managerState']
      if manager_state_frame:
        if manager_state_frame != self._manager_process_frame:
          self._manager_process_frame = manager_state_frame
          self._manager_process_not_running = any(not p.running and p.name not in IGNORE_PROCESSES
                                                  for p in sm['managerState'].processes)
        if self._manager_process_not_running:
          self.events.add(EventName.processNotRunning)

    # Only allow engagement with brake pressed when stopped behind another stopped car
    #speeds = self.sm['longitudinalPlan'].speeds
    #if len(speeds) > 1:
    #  v_future = speeds[-1]
    #else:
    #  v_future = 100.0
    #if CS.brakePressed and v_future >= self.CP.vEgoStarting \
    #  and self.CP.openpilotLongitudinalControl and CS.vEgo < 0.3:
    #  self.events.add(EventName.noTarget)

    # atom
    if self.auto_enabled and not self.no_mdps_mods:
      self.ready_timer += 1 if self.ready_timer < 350 else 350
      self.auto_enable( CS )

  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    if self._cs_has_controls_v_cruise:
      self.CI.CS.controls_v_cruise = self._carstate_controls_v_cruise
    CS = self.CI.update(self.CC, can_strs)

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      if all_valid or self.sm.frame * DT_CTRL > 3.5 or SIMULATION:
        if not self.read_only:
          self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])
        self.initialized = True
        self.params.put_bool("ControlsReady", True)

    # Check for CAN timeout
    if not can_strs:
      self.can_rcv_error_counter += 1
      self.can_rcv_error = True
    else:
      self.can_rcv_error = False

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled:
      for ps in self.sm['pandaStates']:
        if ps.safetyModel not in IGNORED_SAFETY_MODES and not ps.controlsAllowed:
          self.mismatch_counter += 1
          break

    self.distance_traveled += CS.vEgo * DT_CTRL

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    t_speed = 30 if IS_KPH else 20
    m_unit = CV.MS_TO_KPH if IS_KPH else CV.MS_TO_MPH

    if self.v_cruise_kph_set_timer > 0:
      self.v_cruise_kph_set_timer -= 1
    # if stock cruise is completely disabled, then we can use our own set speed logic
    if not self.CP.pcmCruise:
      self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.button_timers, self.enabled, self.is_metric)
    elif self.CP.pcmCruise and CS.cruiseState.enabled:
      if self.variable_cruise and CS.cruiseState.modeSel != 0 and self.CP.vCruisekph > t_speed:
        self.v_cruise_kph = self.CP.vCruisekph
        self.v_cruise_kph_last = self.v_cruise_kph
      elif CS.cruiseButtons == Buttons.RES_ACCEL and self.variable_cruise and CS.cruiseState.modeSel != 0 and CS.vSetDis < (self.v_cruise_kph_last - 1):
        self.v_cruise_kph_set_timer = 30
        self.v_cruise_kph = self.v_cruise_kph_last
        if round(CS.vSetDis)-1 > self.v_cruise_kph:
          self.v_cruise_kph = round(CS.vSetDis)
        self.v_cruise_kph_last = self.v_cruise_kph
      elif CS.cruiseButtons == Buttons.RES_ACCEL and self.variable_cruise and CS.cruiseState.modeSel != 0 and t_speed <= self.v_cruise_kph_last <= round(CS.vEgo*m_unit):
        self.v_cruise_kph_set_timer = 30
        self.v_cruise_kph = round(CS.vEgo*m_unit)
        if round(CS.vSetDis)-1 > self.v_cruise_kph:
          self.v_cruise_kph = round(CS.vSetDis)
        self.v_cruise_kph_last = self.v_cruise_kph
      elif (CS.cruiseButtons == Buttons.RES_ACCEL and not self.v_cruise_kph_set_timer) or CS.cruiseButtons == Buttons.SET_DECEL:
        self.v_cruise_kph = round(CS.cruiseState.speed * m_unit)
        self.v_cruise_kph_last = self.v_cruise_kph
      elif CS.driverAcc and self.variable_cruise and self.cruise_over_maxspeed and t_speed <= self.v_cruise_kph < round(CS.vEgo*m_unit):
        self.v_cruise_kph = round(CS.vEgo*m_unit)
        self.v_cruise_kph_last = self.v_cruise_kph


    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types.clear()
    self.current_alert_types.append(ET.PERMANENT)

    # ENABLED, PRE ENABLING, SOFT DISABLING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.any(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if self.events.any(ET.NO_ENTRY):
            self.state = State.disabled
            self.current_alert_types.append(ET.NO_ENTRY)
          elif not self.events.any(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

    # DISABLED
    elif self.state == State.disabled:
      if self.events.any(ET.ENABLE):
        if self.events.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          #self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)
          self.v_cruise_kph = 0
          self.v_cruise_kph_last = 0

    # Check if actuators are enabled
    self.active = self.state in (State.enabled, State.softDisabling)
    if self.active:
      self.current_alert_types.append(ET.WARNING)

    # Check if openpilot is engaged
    self.enabled = self.active or self.state == State.preEnabled

  def state_control(self, CS):
    """Given the state, this function returns a CarControl packet"""
    sm = self.sm
    lat_plan = sm['lateralPlan']
    long_plan = None if K230_LATERAL_ONLY else sm['longitudinalPlan']

    # opkr
    output_scale = lat_plan.outputScale
    if not self.live_sr:
      if abs(output_scale) >= 1.0 and CS.vEgo > 8 and not CS.steeringPressed:
        self.mpc_frame_sr += 1
        if self.mpc_frame_sr > 20:
          self.new_steerRatio_prev = interp(abs(CS.steeringAngleDeg), self.steer_angle_range, self.steerRatio_range)
          if self.new_steerRatio_prev > self.new_steerRatio:
            self.new_steerRatio = self.new_steerRatio_prev
      else:
        self.mpc_frame += 1
        if self.mpc_frame % 100 == 0:
          self.new_steerRatio -= 0.1
          if self.new_steerRatio <= self.CP.steerRatio:
            self.new_steerRatio = self.CP.steerRatio
          self.mpc_frame = 0
          self.mpc_frame_sr = 0

    # Update VehicleModel
    params = sm['liveParameters']
    x = max(params.stiffnessFactor, 0.1)
    if self.live_sr:
      sr = max(params.steerRatio, 0.1)
      if self.live_sr_percent != 0:
        sr = sr * (1+(0.01*self.live_sr_percent))
    else:
     sr = max(self.new_steerRatio, 0.1)
    if x != self._vm_stiffness or sr != self._vm_steer_ratio:
      self.VM.update_params(x, sr)
      self._vm_stiffness = x
      self._vm_steer_ratio = sr

    self.steerRatio_to_send = sr

    actuators = car.CarControl.Actuators.new_message()
    actuators.longControlState = self.LoC.long_control_state

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = sm.frame

    # State specific actions

    if not self.active:
      self.LaC.reset()
      if not K230_LATERAL_ONLY:
        self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # accel PID loop
      if K230_LATERAL_ONLY:
        actuators.accel = 0.0
        actuators.oaccel = 0.0
      else:
        pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_kph * CV.KPH_TO_MS)
        t_since_plan = (sm.frame - sm.rcv_frame['longitudinalPlan']) * DT_CTRL
        actuators.accel, actuators.oaccel = self.LoC.update(self.active and CS.cruiseState.speed > 1., CS, self.CP, long_plan, pid_accel_limits, t_since_plan, sm['radarState'])

      # Steering PID loop and lateral MPC
      lat_active = self.active and not CS.steerFaultPermanent and not (CS.vEgo < self.CP.minSteerSpeed and self.no_mdps_mods) and not self.lkas_temporary_off
      desired_curvature, desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
                                                                             lat_plan.psis,
                                                                             lat_plan.curvatures,
                                                                             lat_plan.curvatureRates)
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(lat_active, CS, self.CP, self.VM, params, self.last_actuators,
                                                                             desired_curvature, desired_curvature_rate, sm['liveLocationKalman'])
      self.desired_angle_deg = actuators.steeringAngleDeg
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if sm.rcv_frame['testJoystick'] > 0 and self.active:
        actuators.accel = 4.0*clip(sm['testJoystick'].axes[0], -1, 1)

        steer = clip(sm['testJoystick'].axes[1], -1, 1)
        # max angle is 45 for angle-based cars
        actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
        self.desired_angle_deg = actuators.steeringAngleDeg

        lac_log.active = True
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = steer
        lac_log.saturated = abs(steer) >= 0.9

    # Send a "steering required alert" if saturation count has reached the limit
    if lac_log.active and not CS.steeringPressed and self.lat_tuning == 'torque':
      undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 1.2
      turning = abs(lac_log.desiredLateralAccel) > 1.0
      good_speed = CS.vEgo > 5
      max_torque = abs(self.last_actuators.steer) > 0.99
      if undershooting and turning and good_speed and max_torque:
        self.events.add(EventName.steerSaturated)    
    elif lac_log.active and lac_log.saturated and not CS.steeringPressed:
      dpath_points = lat_plan.dPathPoints
      if len(dpath_points):
        # Check if we deviated from the path
        # TODO use desired vs actual curvature
        left_deviation = actuators.steer > 0 and dpath_points[0] < -0.20
        right_deviation = actuators.steer < 0 and dpath_points[0] > 0.20

        if left_deviation or right_deviation:
          self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_NUMERIC_FIELDS:
      attr = getattr(actuators, p)
      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return actuators, lac_log

  def update_button_timers(self, buttonEvents):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in buttonEvents:
      if b.type.raw in self.button_timers:
        self.button_timers[b.type.raw] = 1 if b.pressed else 0

  def publish_logs(self, CS, start_time, actuators, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    sm = self.sm
    sm_frame = sm.frame
    longitudinal_plan = None if K230_LATERAL_ONLY else sm['longitudinalPlan']
    lateral_plan = sm['lateralPlan']
    live_location = sm['liveLocationKalman']
    live_params = sm['liveParameters']
    live_calibration = sm['liveCalibration']
    model_v2 = sm['modelV2']

    self.log_alertTextMsg1 = trace1.global_alertTextMsg1
    self.log_alertTextMsg2 = trace1.global_alertTextMsg2
    self.log_alertTextMsg3 = trace1.global_alertTextMsg3

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled
    CC.active = self.active
    CC.actuators = actuators

    if not K230_LATERAL_ONLY:
      # Orientation and angle rates can be useful for carcontroller
      # Only calibrated (car) frame is relevant for the carcontroller
      orientation_value = live_location.calibratedOrientationNED.value
      if len(orientation_value) > 2:
        CC.orientationNED = list(orientation_value)
      angular_rate_value = live_location.angularVelocityCalibrated.value
      if len(angular_rate_value) > 2:
        CC.angularVelocity = list(angular_rate_value)

    CC.cruiseControl.cancel = self.CP.pcmCruise and not self.enabled and CS.cruiseState.enabled

    if self.joystick_mode and sm.rcv_frame['testJoystick'] > 0 and sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    hudControl = CC.hudControl
    hudControl.setSpeed = float(self.v_cruise_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = False if K230_LATERAL_ONLY else longitudinal_plan.hasLead

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True

    if K230_LATERAL_ONLY:
      v_future = CS.vEgo
      v_future_a = CS.vEgo
    else:
      speeds = longitudinal_plan.speeds # 17 lists
      if len(speeds) > 1:
        v_future = speeds[self.var_cruise_speed_factor]
        v_future_a = speeds[-1]
      else:
        v_future = 100.0
        v_future_a = 100.0
    v_future_speed= float((v_future * CV.MS_TO_KPH) if IS_KPH else (v_future * CV.MS_TO_MPH))
    v_future_speed_a= float((v_future_a * CV.MS_TO_KPH) if IS_KPH else (v_future_a * CV.MS_TO_MPH))
    hudControl.vFuture = v_future_speed
    hudControl.vFutureA = v_future_speed_a

    recent_blinker = (sm_frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                    and not self.active and live_calibration.calStatus == Calibration.CALIBRATED

    desire_prediction = model_v2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = lateral_plan.rProb > 0.5
      left_lane_visible = lateral_plan.lProb > 0.5
      l_lane_change_prob = desire_prediction[Desire.laneChangeLeft - 1]
      r_lane_change_prob = desire_prediction[Desire.laneChangeRight - 1]

      lane_lines = model_v2.laneLines
      if CS.cruiseState.modeSel == 4:
        l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET_A))
        r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET_A))
      else:
        l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
        r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event_types = CLEAR_EVENT_TYPES[(ET.WARNING in self.current_alert_types, self.enabled)]

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, sm, self.is_metric, self.soft_disable_timer])
    self.AM.add_many(sm_frame, alerts)
    current_alert = self.AM.process_alerts(sm_frame, clear_event_types)
    if current_alert:
      hudControl.visualAlert = current_alert.visual_alert

    if self.stock_lkas_on_disengaged_status:
      if self.enabled:
        self.hkg_stock_lkas = False
        self.hkg_stock_lkas_timer = 0
      elif not self.enabled and not self.hkg_stock_lkas:
        self.hkg_stock_lkas_timer += 1
        if self.hkg_stock_lkas_timer > 300:
          self.hkg_stock_lkas = True
          self.hkg_stock_lkas_timer = 0
        elif CS.gearShifter != GearShifter.drive and self.hkg_stock_lkas_timer > 150:
          self.hkg_stock_lkas = True
          self.hkg_stock_lkas_timer = 0
      if not self.hkg_stock_lkas:
        # send car controls over can
        self.last_actuators, can_sends, self.safety_speed, self.lkas_temporary_off, self.gap_by_spd_on_temp = self.CI.apply(CC)
        self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
        CC.actuatorsOutput = self.last_actuators
    else:
      if not self.read_only and self.initialized:
        # send car controls over can
        self.last_actuators, can_sends, self.safety_speed, self.lkas_temporary_off, self.gap_by_spd_on_temp = self.CI.apply(CC)
        self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
        CC.actuatorsOutput = self.last_actuators

    publish_state = True
    car_events = None

    if publish_state:
      force_decel = (self.has_driver_monitoring and sm['driverMonitoringState'].awarenessStatus < 0.) or \
                    (self.state == State.softDisabling)

      # Curvature & Steering angle
      params = live_params

      steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
      curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, params.roll)

      # controlsState
      dat = messaging.new_message('controlsState')
      dat.valid = CS.canValid
      controlsState = dat.controlsState
      if current_alert:
        controlsState.alertText1 = current_alert.alert_text_1
        controlsState.alertText2 = current_alert.alert_text_2
        controlsState.alertSize = current_alert.alert_size
        controlsState.alertStatus = current_alert.alert_status
        controlsState.alertBlinkingRate = current_alert.alert_rate
        controlsState.alertType = current_alert.alert_type
        controlsState.alertSound = current_alert.audible_alert

      controlsState.canMonoTimes = CS.canMonoTimes
      controlsState.lateralPlanMonoTime = sm.logMonoTime['lateralPlan']
      controlsState.longitudinalPlanMonoTime = controlsState.lateralPlanMonoTime if K230_LATERAL_ONLY else sm.logMonoTime['longitudinalPlan']
      controlsState.enabled = self.enabled
      controlsState.active = self.active
      controlsState.curvature = curvature
      controlsState.state = self.state
      controlsState.engageable = not self.events.any(ET.NO_ENTRY)
      if not K230_LATERAL_ONLY:
        controlsState.longControlState = self.LoC.long_control_state
        controlsState.vPid = float(self.LoC.v_pid)
        controlsState.upAccelCmd = float(self.LoC.pid.p)
        controlsState.uiAccelCmd = float(self.LoC.pid.id)
        controlsState.ufAccelCmd = float(self.LoC.pid.f)
      controlsState.vCruise = float(self.v_cruise_kph)
      self._carstate_controls_v_cruise = controlsState.vCruise
      controlsState.cumLagMs = -self.rk.remaining * 1000.
      controlsState.startMonoTime = int(start_time * 1e9)
      controlsState.forceDecel = bool(force_decel)
      controlsState.canErrorCounter = self.can_rcv_error_counter
      controlsState.alertTextMsg1 = self.log_alertTextMsg1
      controlsState.alertTextMsg2 = self.log_alertTextMsg2
      controlsState.alertTextMsg3 = self.log_alertTextMsg3
      controlsState.pauseSpdLimit = self.pause_spdlimit
      controlsState.limitSpeedCamera = 0
      controlsState.limitSpeedCameraDist = 0
      controlsState.mapSign = 0
      controlsState.mapSignCam = 0
      controlsState.lateralControlMethod = int(self.lateral_control_method)
      controlsState.steerRatio = float(self.steerRatio_to_send)
      if not K230_LATERAL_ONLY:
        controlsState.dynamicTRMode = int(longitudinal_plan.dynamicTRMode)
        controlsState.dynamicTRValue = float(longitudinal_plan.dynamicTRValue)
      controlsState.accel = float(self.last_actuators.accel)
      controlsState.steer = float(self.last_actuators.steer * self.steer_max)
      controlsState.safetySpeed = float(self.safety_speed)
      controlsState.gapBySpeedOn = bool(self.gap_by_spd_on_temp)

      lat_tuning = self.lat_tuning
      if self.joystick_mode:
        controlsState.lateralControlState.debugState = lac_log
      elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        controlsState.lateralControlState.angleState = lac_log
      elif lat_tuning == 'pid':
        controlsState.lateralControlState.pidState = lac_log
      elif lat_tuning == 'lqr':
        controlsState.lateralControlState.lqrState = lac_log
      elif lat_tuning == 'indi':
        controlsState.lateralControlState.indiState = lac_log
      elif lat_tuning == 'torque':
        controlsState.lateralControlState.torqueState = lac_log
      elif lat_tuning == 'atom':
        controlsState.lateralControlState.atomState = lac_log

      if lat_tuning == 'torque':
        controlsState.steeringAngleDesiredDeg = lac_log.desiredLateralAccel
      else:
        controlsState.steeringAngleDesiredDeg = self.desired_angle_deg

      self.pm.send('controlsState', dat)

      # carState
      car_events = self.events.to_msg()
      cs_send = messaging.new_message('carState')
      cs_send.valid = CS.canValid
      cs_send.carState = CS
      cs_send.carState.events = car_events
      self.pm.send('carState', cs_send)

    # carEvents - logged every second or on change
    if (sm_frame % self._car_events_interval == 0) or (self.events.names != self.events_prev):
      if car_events is None:
        car_events = self.events.to_msg()
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev[:] = self.events.names

    # carParams - logged every 50 seconds (> 1 per segment)
    if (sm_frame % self._car_params_interval == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # carControl
    if publish_state and self._publish_car_control:
      cc_send = messaging.new_message('carControl')
      cc_send.valid = CS.canValid
      cc_send.carControl = CC
      self.pm.send('carControl', cc_send)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC

  def step(self):
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    self.prof.checkpoint("Sample")

    self.update_events(CS)

    if not self.read_only and self.initialized:
      # Update control state
      self.state_transition(CS)
      self.prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, lac_log = self.state_control(CS)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, actuators, lac_log)
    self.prof.checkpoint("Sent")

    self.update_button_timers(CS.buttonEvents)

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()

def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
