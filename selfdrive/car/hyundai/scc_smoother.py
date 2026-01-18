#!/usr/bin/env python3
"""
SCC 없는 차량을 위한 비전 기반 버튼 스팸 종방향 제어 모듈
K7 하이브리드 등 일반 크루즈만 있는 차량에서 사용

비전 모델(modelV2.leadsV3)의 선행차 인식 데이터를 활용하여
크루즈 버튼 스팸 방식으로 속도를 조절합니다.
"""

from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from selfdrive.car.hyundai.values import Buttons


class SccSmoother:
  """비전 기반 선행차 추종을 위한 버튼 스팸 컨트롤러"""

  def __init__(self):
    # 버튼 상태 머신
    self.btn_cnt = 0
    self.seq_command = 0  # 0: 대기, 1: 가속, 2: 감속, 3: 휴식
    self.target_speed = 0

    # K7 HEV 설정: 버튼 1회 = 2 kph
    self.button_step = 2
    self.min_speed_kph = 30
    self.max_speed_kph = 160

    # 버튼 스팸 설정 (100Hz 기준)
    # 차량 반응 속도를 고려하여 충분한 간격 확보
    self.max_btn_count = 3            # 연속 버튼 전송 횟수 (3회 = 6 kph 변화)
    self.btn_interval_frames = 30     # 버튼 전송 간격 (300ms @ 100Hz) - 차량 반응 대기
    self.btn_rest_frames = 50         # 버튼 휴식 시간 (500ms @ 100Hz) - 속도 안정화 대기
    self.frame_cnt = 0                # 프레임 카운터

    # 안전 파라미터 (일반 크루즈는 브레이크 없이 엔진브레이크만 사용하므로 여유있게 설정)
    self.emergency_dist_m = 12.0  # 긴급 감속 거리 (m)
    self.safe_time_gap = 2.5  # 안전 시간 간격 (초)

    # 디버그
    self.last_button = Buttons.NONE

  def get_button(self, lead, v_ego, v_cruise_set, enabled):
    """
    선행차 정보와 현재 상태로 버튼 신호 결정

    Args:
        lead: radarState.leadOne (비전 기반 선행차 정보)
        v_ego: 현재 속도 (m/s)
        v_cruise_set: 현재 크루즈 설정 속도 (kph)
        enabled: 크루즈 활성화 여부

    Returns:
        Buttons.RES_ACCEL, Buttons.SET_DECEL, 또는 None
    """
    if not enabled:
      self._reset()
      return None

    # 목표 속도 계산
    self.target_speed = self._calc_target_speed(lead, v_ego, v_cruise_set)

    # 버튼 상태 머신 실행
    return self._button_state_machine(v_cruise_set)

  def _calc_target_speed(self, lead, v_ego, v_cruise_set):
    """
    선행차 정보 기반 목표 속도 계산

    Args:
        lead: 선행차 정보 (dRel: 거리, vRel: 상대속도, status: 감지여부)
        v_ego: 현재 속도 (m/s)
        v_cruise_set: 크루즈 설정 속도 (kph)

    Returns:
        목표 속도 (kph)
    """
    # 선행차 없으면 설정 속도 유지
    if not lead.status or lead.dRel <= 0:
      return v_cruise_set

    # 선행차 절대 속도 (kph)
    lead_speed_kph = (v_ego + lead.vRel) * CV.MS_TO_KPH

    # 안전 거리 계산 (Time Gap 기반)
    safe_dist = max(v_ego * self.safe_time_gap, 15.0)  # 최소 15m

    # 긴급 상황: 선행차가 너무 가까우면 즉시 감속
    if lead.dRel < self.emergency_dist_m and lead.vRel < -1.0:
      # 선행차보다 15 kph 느리게 목표 설정 (일반 크루즈는 감속이 느리므로)
      return max(self.min_speed_kph, lead_speed_kph - 15)

    # 거리 비율 계산 (0.0 = 매우 가까움, 1.0 = safe_dist, 2.0 = 충분히 멀리)
    dist_ratio = lead.dRel / safe_dist if safe_dist > 0 else 0

    # 거리에 비례한 속도 마진 계산
    # 가까울수록 선행차보다 느리게, 멀수록 선행차 속도 또는 설정 속도로
    if dist_ratio < 0.5:
      # 매우 가까움: 선행차보다 10~15 kph 느리게 (거리에 비례)
      speed_margin = interp(dist_ratio, [0.0, 0.5], [-15.0, -8.0])
      target = lead_speed_kph + speed_margin
    elif dist_ratio < 0.8:
      # 가까움: 선행차보다 3~8 kph 느리게
      speed_margin = interp(dist_ratio, [0.5, 0.8], [-8.0, -3.0])
      target = lead_speed_kph + speed_margin
    elif dist_ratio < 1.0:
      # 적정 거리 근접: 선행차 속도로
      target = lead_speed_kph
    elif dist_ratio < 1.5:
      # 약간 멀리: 선행차와 설정 속도 사이
      target = min(lead_speed_kph + 5, v_cruise_set)
    else:
      # 충분히 멀리: 설정 속도로
      target = v_cruise_set

    # 설정 속도 초과 방지
    target = min(target, v_cruise_set)

    # 속도 범위 제한
    return clip(target, self.min_speed_kph, min(self.max_speed_kph, v_cruise_set))

  def _button_state_machine(self, v_cruise_set):
    """
    버튼 상태 머신 (navicontrol.py 패턴 기반)

    상태:
      0: 대기 - 목표와 현재 비교하여 가속/감속 결정
      1: 가속 - RES_ACCEL 버튼 전송
      2: 감속 - SET_DECEL 버튼 전송
      3: 휴식 - 버튼 off 유지

    타이밍 (100Hz 기준):
      - 버튼 전송 간격: 300ms (btn_interval_frames=30) - 차량 반응 대기
      - 연속 버튼: 최대 3회 (900ms 동안)
      - 휴식 시간: 500ms (btn_rest_frames=50) - 속도 안정화 대기
      - 한 사이클: 약 1.4초, 속도 변화 6 kph

    Returns:
        버튼 신호 또는 None
    """
    self.frame_cnt += 1
    return self._switch(self.seq_command, v_cruise_set)

  def _switch(self, seq_cmd, v_cruise_set):
    """상태별 처리 함수 호출"""
    if seq_cmd == 0:
      return self._case_0(v_cruise_set)
    elif seq_cmd == 1:
      return self._case_1(v_cruise_set)
    elif seq_cmd == 2:
      return self._case_2(v_cruise_set)
    elif seq_cmd == 3:
      return self._case_3()
    return None

  def _case_0(self, v_cruise_set):
    """대기 상태: 목표와 현재 비교"""
    self.btn_cnt = 0
    delta = round(self.target_speed - v_cruise_set)

    # 2 kph 단위이므로 +-1 kph 이내면 조정 불필요
    if delta >= self.button_step:
      self.seq_command = 1  # 가속으로 전환
      self.frame_cnt = self.btn_interval_frames  # 즉시 첫 버튼 전송 가능하도록
    elif delta <= -self.button_step:
      self.seq_command = 2  # 감속으로 전환
      self.frame_cnt = self.btn_interval_frames  # 즉시 첫 버튼 전송 가능하도록
    return None

  def _case_1(self, v_cruise_set):
    """가속 상태: RES_ACCEL 버튼 전송 (간격 제어)"""
    # 버튼 전송 간격 확인 (CAN 오류 방지)
    if self.frame_cnt < self.btn_interval_frames:
      return None  # 아직 전송 간격 미달

    self.frame_cnt = 0  # 프레임 카운터 리셋
    self.btn_cnt += 1
    self.last_button = Buttons.RES_ACCEL

    # 목표 도달 또는 최대 횟수 도달 시 휴식으로
    if self.target_speed <= v_cruise_set or self.btn_cnt >= self.max_btn_count:
      self.btn_cnt = 0
      self.seq_command = 3
    return Buttons.RES_ACCEL

  def _case_2(self, v_cruise_set):
    """감속 상태: SET_DECEL 버튼 전송 (간격 제어)"""
    # 버튼 전송 간격 확인 (CAN 오류 방지)
    if self.frame_cnt < self.btn_interval_frames:
      return None  # 아직 전송 간격 미달

    self.frame_cnt = 0  # 프레임 카운터 리셋
    self.btn_cnt += 1
    self.last_button = Buttons.SET_DECEL

    # 목표 도달 또는 최대 횟수 도달 시 휴식으로
    if self.target_speed >= v_cruise_set or self.btn_cnt >= self.max_btn_count:
      self.btn_cnt = 0
      self.seq_command = 3
    return Buttons.SET_DECEL

  def _case_3(self):
    """휴식 상태: 버튼 off 유지"""
    self.last_button = Buttons.NONE

    # 휴식 시간 후 대기 상태로 복귀
    if self.frame_cnt >= self.btn_rest_frames:
      self.frame_cnt = 0
      self.seq_command = 0
    return None

  def _reset(self):
    """상태 초기화"""
    self.btn_cnt = 0
    self.seq_command = 0
    self.target_speed = 0
    self.frame_cnt = 0
    self.last_button = Buttons.NONE

  def get_lead_info(self, lead):
    """디버그용 선행차 정보 반환"""
    if lead.status and lead.dRel > 0:
      return {
        'dist': lead.dRel,
        'rel_speed': lead.vRel * CV.MS_TO_KPH,
        'status': True
      }
    return {'dist': 0, 'rel_speed': 0, 'status': False}
