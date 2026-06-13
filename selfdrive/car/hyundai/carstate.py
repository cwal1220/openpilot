from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES, EV_CAR, HYBRID_CAR, Buttons, CAR
from selfdrive.car.interfaces import CarStateBase
from common.numpy_fast import interp
from common.params import Params

GearShifter = car.CarState.GearShifter

FCA_OPT = Params().get_bool('RadarDisable')

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    params = Params()
    self.params = params
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    #Auto detection for setup
    self.no_radar = CP.sccBus == -1
    self.lkas_button_on = True
    self.cruise_main_button = 0
    self.mdps_error_cnt = 0
    self.cruiseState_standstill = False

    self.lfahda = None

    self.driverAcc_time = 0

    self.prev_cruise_buttons = 0
    self.prev_gap_button = 0
    
    self.steer_anglecorrection = float(int(params.get("OpkrSteerAngleCorrection", encoding="utf8")) * 0.1)
    self.gear_correction = params.get_bool("JustDoGearD")
    self.fca11_message = params.get_bool("FCA11Message")
    self.rd_conf = params.get_bool("RadarDisable")
    self.set_spd_five = params.get_bool("SetSpeedFive")
    self.brake_check = False
    self.cancel_check = False
    
    self.cruise_gap = 4
    self.is_set_speed_in_mph = False
    self.cruise_active = False

    # atom
    self.cruise_buttons = 0
    self.cruise_buttons_time = 0
    self.time_delay_int = 0
    self.VSetDis = 0
    self.clu_Vanz = 0

    # acc button 
    self.prev_clu_CruiseSwState = 0
    self.prev_acc_active = False
    self.prev_acc_set_btn = False
    self.prev_cruise_btn = False
    self.acc_active = False
    self.cruise_set_speed_kph = 0
    self.cruise_set_mode = int(params.get("CruiseStatemodeSelInit", encoding="utf8"))
    self.gasPressed = False

    self.controls_v_cruise = 0
    self.wheel_speed_factor = CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    self.is_hybrid = self.CP.carFingerprint in HYBRID_CAR
    self.is_ev = self.CP.carFingerprint in EV_CAR
    self.use_cluster_gears = self.CP.carFingerprint in FEATURES["use_cluster_gears"]
    self.use_tcu_gears = self.CP.carFingerprint in FEATURES["use_tcu_gears"]
    self.use_elect_gears = self.CP.carFingerprint in FEATURES["use_elect_gears"]
    self.use_fca = self.CP.carFingerprint in FEATURES["use_fca"]
    self.send_hda_mfa = self.CP.carFingerprint in FEATURES["send_hda_mfa"]
    self.has_scc = self.CP.sccBus != -1
    self.bsm_available = self.CP.bsmAvailable
    self.openpilot_longitudinal = self.CP.openpilotLongitudinalControl

  def set_cruise_speed(self, set_speed):
    self.cruise_set_speed_kph = set_speed

  #@staticmethod
  def cruise_speed_button(self, controls_v_cruise):
    set_speed_kph = self.cruise_set_speed_kph
    controls_v_cruise_rounded = round(controls_v_cruise)
    if 1 < controls_v_cruise_rounded < 255:
      set_speed_kph = controls_v_cruise_rounded

    if self.cruise_buttons:
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0

    # long press should set scc speed with cluster scc number
    if self.cruise_buttons_time >= 60:
      self.cruise_set_speed_kph = self.VSetDis
      return self.cruise_set_speed_kph

    if self.prev_cruise_btn == self.cruise_buttons:
      return self.cruise_set_speed_kph
    elif self.prev_cruise_btn != self.cruise_buttons:
      self.prev_cruise_btn = self.cruise_buttons
      if not self.cruise_active:
        if self.cruise_buttons == Buttons.GAP_DIST:  # mode change
          self.cruise_set_mode += 1
          if self.cruise_set_mode > 5:
            self.cruise_set_mode = 0
          return None
        elif not self.prev_acc_set_btn: # first scc active
          self.prev_acc_set_btn = self.acc_active
          if self.cruise_buttons == Buttons.SET_DECEL:
            self.cruise_set_speed_kph = max(int(round(self.clu_Vanz)), (30 if not self.is_set_speed_in_mph else 20))
          elif self.cruise_buttons == Buttons.RES_ACCEL:
            self.cruise_set_speed_kph = max(set_speed_kph, int(round(self.clu_Vanz)), (30 if not self.is_set_speed_in_mph else 20))
          return self.cruise_set_speed_kph

      elif self.cruise_buttons == Buttons.RES_ACCEL and not self.cruiseState_standstill:   # up 
        if self.set_spd_five:
          set_speed_kph += 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph += 1
      elif self.cruise_buttons == Buttons.SET_DECEL and not self.cruiseState_standstill:  # dn
        if self.set_spd_five:
          set_speed_kph -= 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph -= 1

      if set_speed_kph <= 30 and not self.is_set_speed_in_mph:
        set_speed_kph = 30
      elif set_speed_kph <= 20 and self.is_set_speed_in_mph:
        set_speed_kph = 20

      self.cruise_set_speed_kph = set_speed_kph
    else:
      self.prev_cruise_btn = False

    return set_speed_kph

  def update(self, cp, cp2, cp_cam):
    cp_mdps = cp2 if self.CP.mdpsBus == 1 else cp
    cp_sas = cp2 if self.CP.sasBus else cp
    cp_scc = cp_cam if self.CP.sccBus == 2 else cp2 if self.CP.sccBus == 1 else cp
    cp_fca = cp_cam if (self.CP.fcaBus == 2) else cp

    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_lkas_button_on = self.lkas_button_on

    ret = car.CarState.new_message()

    cp_vl = cp.vl
    cp_mdps_vl = cp_mdps.vl
    cp_sas_vl = cp_sas.vl
    cp_scc_vl = cp_scc.vl
    cp_cam_vl = cp_cam.vl
    cp_fca_vl = cp_fca.vl

    cgw1 = cp_vl["CGW1"]
    cgw2 = cp_vl["CGW2"]
    clu11 = cp_vl["CLU11"]
    esp12 = cp_vl["ESP12"]
    tcs13 = cp_vl["TCS13"]
    tcs15 = cp_vl["TCS15"]
    whl_spd11 = cp_vl["WHL_SPD11"]
    lvr12 = cp_vl["LVR12"]
    tpms11 = cp_vl["TPMS11"]
    mdps12 = cp_mdps_vl["MDPS12"]
    sas11 = cp_sas_vl["SAS11"]
    scc11 = cp_scc_vl["SCC11"]
    scc12 = cp_scc_vl["SCC12"]
    lkas11 = cp_cam_vl["LKAS11"]
    cruise_state = ret.cruiseState

    ret.doorOpen = bool(cgw1["CF_Gway_DrvDrSw"] or cgw1["CF_Gway_AstDrSw"] or
                        cgw2["CF_Gway_RLDrSw"] or cgw2["CF_Gway_RRDrSw"])

    ret.seatbeltUnlatched = cgw1["CF_Gway_DrvSeatBeltSw"] == 0

    wheel_speed_factor = self.wheel_speed_factor
    wheel_speed_fl = whl_spd11["WHL_SPD_FL"] * wheel_speed_factor
    wheel_speed_fr = whl_spd11["WHL_SPD_FR"] * wheel_speed_factor
    wheel_speed_rl = whl_spd11["WHL_SPD_RL"] * wheel_speed_factor
    wheel_speed_rr = whl_spd11["WHL_SPD_RR"] * wheel_speed_factor
    ret.wheelSpeeds.fl = wheel_speed_fl
    ret.wheelSpeeds.fr = wheel_speed_fr
    ret.wheelSpeeds.rl = wheel_speed_rl
    ret.wheelSpeeds.rr = wheel_speed_rr
    v_ego_raw = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) / 4.
    ret.vEgoRaw = v_ego_raw
    v_ego_op, ret.aEgo = self.update_speed_kf(v_ego_raw)
    ret.vEgoOP = v_ego_op

    ret.vEgo = clu11["CF_Clu_Vanz"] * CV.MPH_TO_MS if bool(clu11["CF_Clu_SPEED_UNIT"]) else clu11["CF_Clu_Vanz"] * CV.KPH_TO_MS

    ret.standstill = ret.vEgoRaw < 0.1
    ret.standStill = self.CP.standStill

    ret.steeringAngleDeg = sas11["SAS_Angle"] - self.steer_anglecorrection
    ret.steeringRateDeg = sas11["SAS_Speed"]
    ret.yawRate = esp12["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cgw1["CF_Gway_TurnSigLh"], cgw1["CF_Gway_TurnSigRh"])
    ret.steeringTorque = mdps12["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = mdps12["CR_Mdps_OutTq"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    self.mdps_error_cnt += 1 if mdps12["CF_Mdps_ToiUnavail"] != 0 else -self.mdps_error_cnt
    ret.steerFaultTemporary = self.mdps_error_cnt > 100 #cp_mdps.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0

    self.VSetDis = scc11["VSetDis"]
    ret.vSetDis = self.VSetDis
    self.clu_Vanz = clu11["CF_Clu_Vanz"]
    lead_objspd = scc11["ACC_ObjRelSpd"]
    self.lead_objspd = lead_objspd * CV.MS_TO_KPH
    self.Mdps_ToiUnavail = mdps12["CF_Mdps_ToiUnavail"]
    self.driverOverride = tcs13["DriverOverride"]
    if self.driverOverride == 1:
      self.driverAcc_time = 100
    elif self.driverAcc_time:
      self.driverAcc_time -= 1

    # cruise state, 크루즈 여부와 관련 없이 무조건 활성화
    cruise_state.enabled = True
    cruise_state.available = True

    cruise_state.standstill = scc11["SCCInfoDisplay"] == 4. if not self.no_radar else False
    self.cruiseState_standstill = cruise_state.standstill
    self.is_set_speed_in_mph = bool(clu11["CF_Clu_SPEED_UNIT"])
    ret.isMph = self.is_set_speed_in_mph
    
    self.acc_active = (scc12['ACCMode'] != 0)
    self.cruise_active = self.acc_active
    if self.cruise_active:
      self.brake_check = False
      self.cancel_check = False

    cruise_state.accActive = self.acc_active
    cruise_state.gapSet = scc11['TauGapSet']
    cruise_state.cruiseSwState = self.cruise_buttons
    cruise_state.modeSel = self.cruise_set_mode

    set_speed = self.cruise_speed_button(self.controls_v_cruise)
    if cruise_state.enabled and (self.brake_check == False or self.cancel_check == False):
      speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
      cruise_state.speed = set_speed * speed_conv if not self.no_radar else \
                           lvr12["CF_Lvr_CruiseSet"] * speed_conv
    else:
      cruise_state.speed = 0

    self.cruise_main_button = clu11["CF_Clu_CruiseSwMain"]
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = clu11["CF_Clu_CruiseSwState"]
    ret.cruiseButtons = self.cruise_buttons

    if self.prev_gap_button != self.cruise_buttons:
      if self.cruise_buttons == 3:
        self.cruise_gap -= 1
      if self.cruise_gap < 1:
        self.cruise_gap = 4
      self.prev_gap_button = self.cruise_buttons

    # TODO: Find brake pressure
    ret.brake = 0
    brake_pressed = tcs13["DriverBraking"] != 0
    ret.brakePressed = brake_pressed

    if brake_pressed:
      self.brake_check = True
    if self.cruise_buttons == 4:
      self.cancel_check = True

    # TODO: Check this
    ret.brakeLights = bool(tcs13["BrakeLight"] or brake_pressed)

    if self.is_hybrid or self.is_ev:
      if self.is_hybrid:
        e_ems11 = cp_vl["E_EMS11"]
        ret.gas = e_ems11["CR_Vcu_AccPedDep_Pos"] / 254.
        ret.engineRpm = e_ems11["N"] # opkr
        ret.chargeMeter = 0
      else:
        e_ems11 = cp_vl["E_EMS11"]
        elect_gear = cp_vl["ELECT_GEAR"]
        ret.gas = e_ems11["Accel_Pedal_Pos"] / 254.
        ret.engineRpm = elect_gear["Elect_Motor_Speed"] * 30 # opkr, may multiply deceleration ratio in line with engine rpm
        ret.chargeMeter = cp_vl["EV_Info"]["OPKR_EV_Charge_Level"] # opkr
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp_vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp_vl["EMS16"]["CF_Ems_AclAct"])
      ret.engineRpm = cp_vl["EMS_366"]["N"]
      ret.chargeMeter = 0

    ret.espDisabled = (tcs15["ESC_Off_Step"] != 0)

    self.parkBrake = tcs13["PBRAKE_ACT"] == 1
    gas_pressed = ret.gasPressed
    self.gasPressed = gas_pressed

    # opkr
    tpms_unit = tpms11["UNIT"]
    tpms_factor = 0.72519 if tpms_unit == 1 else 0.1 if tpms_unit == 2 else 1 # 0:psi, 1:kpa, 2:bar
    tpms = ret.tpms
    tpms.unit = tpms_unit
    tpms.fl = tpms11["PRESSURE_FL"] * tpms_factor
    tpms.fr = tpms11["PRESSURE_FR"] * tpms_factor
    tpms.rl = tpms11["PRESSURE_RL"] * tpms_factor
    tpms.rr = tpms11["PRESSURE_RR"] * tpms_factor

    ret.safetySign = 0.
    ret.safetyDist = 0.
    self.cruiseGapSet = scc11["TauGapSet"]
    ret.cruiseGapSet = self.cruiseGapSet

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.use_cluster_gears:
      gear = cp_vl["CLU15"]["CF_Clu_Gear"]
      ret.gearStep = 0
    elif self.use_tcu_gears:
      gear = cp_vl["TCU12"]["CUR_GR"]
      ret.gearStep = 0
    elif self.use_elect_gears:
      if self.CP.carFingerprint == CAR.NEXO_FE:
        gear = cp_vl["EMS20"]["Elect_Gear_Shifter_NEXO"] # NEXO gear by multikyd
      else:
        gear = cp_vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      ret.gearStep = cp_vl["ELECT_GEAR"]["Elect_Gear_Step"] # opkr
    else:
      gear = lvr12["CF_Lvr_Gear"]
      ret.gearStep = cp_vl["LVR11"]["CF_Lvr_GearInf"] # opkr

    if self.gear_correction:
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.has_scc:
      if self.use_fca or self.fca11_message:
        fca11 = cp_fca_vl["FCA11"]
        ret.stockAeb = fca11["FCA_CmdAct"] != 0
        ret.stockFcw = fca11["CF_VSM_Warn"] == 2
      else:
        ret.stockAeb = scc12["AEB_CmdAct"] != 0
        ret.stockFcw = scc12["CF_VSM_Warn"] == 2

    # Blind Spot Detection and Lane Change Assist signals
    if self.bsm_available:
      lca11 = cp_vl["LCA11"]
      ret.leftBlindspot = lca11["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = lca11["CF_Lca_IndRight"] != 0

    # save the entire LKAS11, CLU11, SCC12 and MDPS12 for CarController
    self.lkas11 = lkas11.copy()
    self.clu11 = clu11.copy()
    self.scc11 = scc11.copy()
    self.scc12 = scc12.copy()
    self.mdps12 = mdps12.copy()

    if self.openpilot_longitudinal:
      self.scc13 = cp_scc_vl["SCC13"].copy()
      self.scc14 = cp_scc_vl["SCC14"].copy()
      self.scc11init = cp_vl["SCC11"].copy()
      self.scc12init = cp_vl["SCC12"].copy()
      if self.rd_conf:
        self.fca11 = cp_fca_vl["FCA11"].copy()
        self.fca11init = cp_vl["FCA11"].copy()

    if self.send_hda_mfa:
      self.lfahda = cp_cam_vl["LFAHDA_MFC"].copy()

    brake_hold = tcs15["AVH_LAMP"] == 2 # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.brakeHold = brake_hold
    self.brakeHold = brake_hold
    self.brake_error = tcs13["ACCEnable"] == 3 # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    self.steer_state = mdps12["CF_Mdps_ToiActive"] #0 NOT ACTIVE, 1 ACTIVE
    self.lead_distance = scc11["ACC_ObjDist"] if not self.no_radar else 0

    ret.radarDistance = scc11["ACC_ObjDist"] if not self.no_radar else 0
    self.lkas_error = lkas11["CF_Lkas_LdwsSysState"] == 7
    if not self.lkas_error:
      self.lkas_button_on = lkas11["CF_Lkas_LdwsSysState"]
    
    ret.cruiseAccStatus = scc12["ACCMode"] == 1
    ret.driverAcc = self.driverOverride == 1
    ret.aReqValue = scc12["aReqValue"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # signal_name, signal_address
      ("WHL_SPD_FL", "WHL_SPD11"),
      ("WHL_SPD_FR", "WHL_SPD11"),
      ("WHL_SPD_RL", "WHL_SPD11"),
      ("WHL_SPD_RR", "WHL_SPD11"),

      ("YAW_RATE", "ESP12"),

      ("CF_Gway_DrvSeatBeltInd", "CGW4"),

      ("CF_Gway_DrvSeatBeltSw", "CGW1"),
      ("CF_Gway_DrvDrSw", "CGW1"),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1"),       # Passenger door
      ("CF_Gway_RLDrSw", "CGW2"),        # Rear left door
      ("CF_Gway_RRDrSw", "CGW2"),        # Rear right door
      ("CF_Gway_TurnSigLh", "CGW1"),
      ("CF_Gway_TurnSigRh", "CGW1"),
      ("CF_Gway_ParkBrakeSw", "CGW1"),

      ("CYL_PRES", "ESP12"),

      ("AVH_STAT", "ESP11"),

      ("CF_Clu_CruiseSwState", "CLU11"),
      ("CF_Clu_CruiseSwMain", "CLU11"),
      ("CF_Clu_SldMainSW", "CLU11"),
      ("CF_Clu_ParityBit1", "CLU11"),
      ("CF_Clu_VanzDecimal" , "CLU11"),
      ("CF_Clu_Vanz", "CLU11"),
      ("CF_Clu_SPEED_UNIT", "CLU11"),
      ("CF_Clu_DetentOut", "CLU11"),
      ("CF_Clu_RheostatLevel", "CLU11"),
      ("CF_Clu_CluInfo", "CLU11"),
      ("CF_Clu_AmpInfo", "CLU11"),
      ("CF_Clu_AliveCnt1", "CLU11"),

      ("ACCEnable", "TCS13"),
      ("BrakeLight", "TCS13"),
      ("DriverBraking", "TCS13"),
      ("DriverOverride", "TCS13"),
      ("PBRAKE_ACT", "TCS13"),
      ("CF_VSM_Avail", "TCS13"),

      ("ESC_Off_Step", "TCS15"),
      ("AVH_LAMP", "TCS15"),

      ("CF_Lvr_CruiseSet", "LVR12"),
      ("CRUISE_LAMP_M", "EMS16"),

      ("MainMode_ACC", "SCC11"),
      ("SCCInfoDisplay", "SCC11"),
      ("AliveCounterACC", "SCC11"),
      ("VSetDis", "SCC11"),
      ("ObjValid", "SCC11"),
      ("DriverAlertDisplay", "SCC11"),
      ("TauGapSet", "SCC11"),
      ("ACC_ObjStatus", "SCC11"),
      ("ACC_ObjLatPos", "SCC11"),
      ("ACC_ObjDist", "SCC11"), #TK211X value is 204.6
      ("ACC_ObjRelSpd", "SCC11"),
      ("ACCMode", "SCC12"),
      ("CF_VSM_Prefill", "SCC12"),
      ("CF_VSM_DecCmdAct", "SCC12"),
      ("CF_VSM_HBACmd", "SCC12"),
      ("CF_VSM_Warn", "SCC12"),
      ("CF_VSM_Stat", "SCC12"),
      ("CF_VSM_BeltCmd", "SCC12"),
      ("ACCFailInfo", "SCC12"),
      ("StopReq", "SCC12"),
      ("CR_VSM_DecCmd", "SCC12"),
      ("aReqRaw", "SCC12"), #aReqMax
      ("TakeOverReq", "SCC12"),
      ("PreFill", "SCC12"),
      ("aReqValue", "SCC12"), #aReqMin
      ("CF_VSM_ConfMode", "SCC12"),
      ("AEB_Failinfo", "SCC12"),
      ("AEB_Status", "SCC12"),
      ("AEB_CmdAct", "SCC12"),
      ("AEB_StopReq", "SCC12"),
      ("CR_VSM_Alive", "SCC12"),
      ("CR_VSM_ChkSum", "SCC12"),

      ("SCCDrvModeRValue", "SCC13"),
      ("SCC_Equip", "SCC13"),
      ("AebDrvSetStatus", "SCC13"),

      ("JerkUpperLimit", "SCC14"),
      ("JerkLowerLimit", "SCC14"),
      ("SCCMode2", "SCC14"),
      ("ComfortBandUpper", "SCC14"),
      ("ComfortBandLower", "SCC14"),

      ("CR_FCA_Alive", "FCA11"),
      ("Supplemental_Counter", "FCA11"),

      ("UNIT", "TPMS11"),
      ("PRESSURE_FL", "TPMS11"),
      ("PRESSURE_FR", "TPMS11"),
      ("PRESSURE_RL", "TPMS11"),
      ("PRESSURE_RR", "TPMS11"),

      ("N", "EMS_366"),

      ("OPKR_EV_Charge_Level", "EV_Info")
    ]

    checks = [
      # address, frequency
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50)
    ]
    if CP.sccBus == 0 and CP.pcmCruise:
      checks += [
        ("SCC11", 50),
        ("SCC12", 50)
      ]
    if CP.fcaBus == 0:
      signals.append(("CR_Vcu_AccPedDep_Pos", "E_EMS11"))
      signals += [
        ("FCA_CmdAct", "FCA11"),
        ("CF_VSM_Warn", "FCA11")
      ]
      checks += [("FCA11", 50)]

    if CP.mdpsBus == 0:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [("MDPS12", 50)]

    if CP.sasBus == 0:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11")
      ]
      checks += [("SAS11", 100)]

    if CP.bsmAvailable:
      signals += [
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11")
      ]
      checks += [("LCA11", 50)]

    if CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if CP.carFingerprint in HYBRID_CAR:
        signals += [
          ("CR_Vcu_AccPedDep_Pos", "E_EMS11"),
          ("N", "E_EMS11")
        ]
      else:
        signals += [("Accel_Pedal_Pos", "E_EMS11")]
      checks += [("E_EMS11", 50)]
    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("CF_Ems_AclAct", "EMS16")
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100)
      ]

    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [("CF_Clu_Gear", "CLU15")]
      checks += [("CLU15", 5)]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [("CUR_GR", "TCU12")]
      checks += [("TCU12", 100)]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [
        ("Elect_Gear_Shifter", "ELECT_GEAR"),
        ("Elect_Gear_Step", "ELECT_GEAR"),
        ("Elect_Motor_Speed", "ELECT_GEAR")
      ]
      checks += [("ELECT_GEAR", 20)]
      if CP.carFingerprint == CAR.NEXO_FE:
        signals += [("Elect_Gear_Shifter_NEXO", "EMS20")]
        checks += [("EMS20", 20)]
    else:
      signals += [
        ("CF_Lvr_Gear", "LVR12"),
        ("CF_Lvr_GearInf", "LVR11")
      ]
      checks += [
        ("LVR12", 100),
        ("LVR11", 100)
      ]

    if CP.carFingerprint == CAR.SANTAFE_TM:
      checks.remove(("TCS13", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0, enforce_checks=False, track_all=False)

  @staticmethod
  def get_can2_parser(CP):
    signals = []
    checks = []
    if CP.mdpsBus == 1:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [("MDPS12", 50)]
    if CP.sasBus == 1:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11")
      ]
      checks += [("SAS11", 100)]
    if CP.sccBus == 1:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14")
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50)
      ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1, enforce_checks=False, track_all=False)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_ToiFlt", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_MsgCount", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11")
    ]
    checks = [("LKAS11", 100)]
    if CP.sccBus == 2:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"),
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"),
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),
        ("ACCMode", "SCC14"),
        ("ObjGap", "SCC14")
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50)
      ]
      if CP.fcaBus == 2:
        signals += [
          ("CF_VSM_Prefill", "FCA11"),
          ("CF_VSM_HBACmd", "FCA11"),
          ("CF_VSM_Warn", "FCA11"),
          ("CF_VSM_BeltCmd", "FCA11"),
          ("CR_VSM_DecCmd", "FCA11"),
          ("FCA_Status", "FCA11"),
          ("FCA_CmdAct", "FCA11"),
          ("FCA_StopReq", "FCA11"),
          ("FCA_DrvSetStatus", "FCA11"),
          ("CF_VSM_DecCmdAct", "FCA11"),
          ("FCA_Failinfo", "FCA11"),
          ("FCA_RelativeVelocity", "FCA11"),
          ("FCA_TimetoCollision", "FCA11"),
          ("CR_FCA_Alive", "FCA11"),
          ("CR_FCA_ChkSum", "FCA11"),
          ("Supplemental_Counter", "FCA11"),
          ("PAINT1_Status", "FCA11")
        ]
        checks += [("FCA11", 50)]

      if CP.carFingerprint in FEATURES["send_hda_mfa"]:
        signals += [
          ("HDA_USM", "LFAHDA_MFC"),
          ("HDA_Active", "LFAHDA_MFC"),
          ("HDA_Icon_State", "LFAHDA_MFC"),
          ("HDA_LdwSysState", "LFAHDA_MFC"),
          ("HDA_Icon_Wheel", "LFAHDA_MFC")
        ]
        checks += [("LFAHDA_MFC", 20)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2, enforce_checks=False, track_all=False)
