# K7 하이브리드 비전 기반 버튼 스팸 종방향 제어 구현 계획

## 개요

**목표**: SCC/레이더 없는 K7 하이브리드에서 비전 기반 선행차 인식과 크루즈 버튼 스팸을 이용한 종방향 제어 구현

**차량 정보** (사용자 확인):
- 크루즈 타입: 일반 크루즈 (정속 주행만 가능, SCC/레이더 없음)
- 버튼 단위: **2 kph / 버튼**
- 정차 기능: 불필요 (일반 크루즈는 정차 시 자동 해제)

**핵심 아이디어**:
- modelV2.leadsV3 (비전 기반 선행차 감지) → 목표 속도 계산 → CLU11 버튼 스팸으로 크루즈 속도 조절

## 가능성 평가: **구현 가능**

기존 OPKR 코드에 이미 다음이 구현되어 있음:
1. ✅ 비전 기반 lead 추적 (`radard.py`: `get_RadarState_from_vision()`)
2. ✅ 버튼 스팸 패턴 (`navicontrol.py`: `case_1/2/3`)
3. ✅ MPC 기반 목표 속도 계산 (`long_mpc.py`)
4. ✅ CLU11 메시지 생성 (`hyundaican.py`: `create_clu11()`)

---

## 수정 파일 목록

| 파일 | 변경 내용 |
|------|----------|
| `selfdrive/car/hyundai/interface.py` | SCC 없음 감지 및 비전 종방향 제어 활성화 |
| `selfdrive/car/hyundai/carstate.py` | SCC 없을 때 크루즈 상태 감지 |
| `selfdrive/car/hyundai/carcontroller.py` | 비전 기반 버튼 스팸 로직 통합 |
| `selfdrive/car/hyundai/scc_smoother.py` | **새 파일**: 비전 기반 목표 속도 계산 및 버튼 제어 |

---

## 구현 단계

### 1단계: interface.py - SCC 없음 감지

```python
# 라인 41-52 근처 수정
# K7 HEV에서 SCC 없는 경우 강제 설정
no_scc_car = candidate == CAR.K7_HEV_YG and not Params().get_bool("HasSCC")

if no_scc_car:
    ret.sccBus = -1
    ret.radarOffCan = True
    ret.openpilotLongitudinalControl = True
    ret.pcmCruise = False  # openpilot이 크루즈 속도 제어
```

### 2단계: carstate.py - 크루즈 상태 감지

SCC 없을 때 `LVR12.CF_Lvr_CruiseSet`과 `EMS16.CRUISE_LAMP_M` 사용:

```python
# SCC 없을 때 크루즈 활성화 감지 (라인 235 근처)
if self.no_radar:
    cruise_set_speed = cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * speed_conv
    cruise_lamp = cp.vl["EMS16"]["CRUISE_LAMP_M"]
    self.cruise_active = cruise_lamp != 0 and cruise_set_speed > 0
```

### 3단계: scc_smoother.py - 비전 기반 버튼 제어 (새 파일)

기존 `navicontrol.py` 패턴을 활용한 새 모듈:

```python
class SccSmoother:
    """비전 기반 선행차 추종을 위한 버튼 스팸 컨트롤러"""

    def __init__(self):
        self.btn_cnt = 0
        self.seq_command = 0
        self.target_speed = 0
        self.button_step = 2  # K7 HEV: 2 kph/버튼

    def get_button(self, lead, v_ego, v_cruise_set, enabled):
        """
        선행차 정보와 현재 상태로 버튼 신호 결정

        Args:
            lead: radarState.leadOne (비전 기반)
            v_ego: 현재 속도 (m/s)
            v_cruise_set: 현재 설정 속도 (kph)
            enabled: 크루즈 활성화 여부

        Returns:
            Buttons.RES_ACCEL, Buttons.SET_DECEL, 또는 None
        """
        if not enabled:
            return None

        # 목표 속도 계산
        target = self._calc_target_speed(lead, v_ego, v_cruise_set)

        # 버튼 상태 머신 (navicontrol.py case_0/1/2/3 패턴)
        return self._button_state_machine(target, v_cruise_set)
```

**목표 속도 계산 로직**:
```python
def _calc_target_speed(self, lead, v_ego, v_cruise_set):
    v_ego_kph = v_ego * CV.MS_TO_KPH

    if not lead.status or lead.dRel <= 0:
        # 선행차 없음: 설정 속도 유지
        return v_cruise_set

    # 선행차 속도
    lead_speed_kph = (v_ego + lead.vRel) * CV.MS_TO_KPH

    # 안전 거리 기반 목표 속도 (Time Gap 2초 기준)
    safe_dist = v_ego * 2.0  # 2초 간격

    if lead.dRel < safe_dist:
        # 너무 가까움: 선행차보다 느리게
        target = min(lead_speed_kph - 5, v_cruise_set)
    elif lead.dRel < safe_dist * 1.5:
        # 적정 거리: 선행차 속도로
        target = min(lead_speed_kph, v_cruise_set)
    else:
        # 멀리 있음: 설정 속도로
        target = v_cruise_set

    return max(30, target)  # 최소 30 kph
```

### 4단계: carcontroller.py - 통합

```python
# 상단 import
from selfdrive.car.hyundai.scc_smoother import SccSmoother

class CarController:
    def __init__(self, ...):
        # SCC 없는 차량용 비전 버튼 제어
        self.scc_smoother = SccSmoother() if CP.radarOffCan else None

    def update(self, ...):
        # 비전 기반 버튼 스팸 (SCC 없을 때)
        if self.scc_smoother and CS.cruise_active:
            lead = self.sm['radarState'].leadOne
            btn = self.scc_smoother.get_button(
                lead, CS.out.vEgo, CS.VSetDis, enabled
            )
            if btn and self.button_wait == 0:
                can_sends.append(create_clu11(
                    self.packer, frame, CS.clu11, btn
                ))
                self.button_wait = 35  # 350ms 대기
```

---

## 버튼 스팸 타이밍

```
[버튼] x 5회 연속 → [대기 7~9프레임] → 반복

- 100Hz 기준 (10ms/프레임)
- 버튼 5회 = 50ms
- 대기 = 70~90ms
- 한 사이클 ≈ 140ms
- 속도 변화: 10 kph (2 kph × 5회)
```

---

## 안전 메커니즘

1. **긴급 감속**: `lead.dRel < 8m && lead.vRel < -2 m/s` → 즉시 SET_DECEL 연속 전송
2. **최소 속도**: 30 kph 이하로 설정하지 않음
3. **운전자 개입**: 브레이크/가속 페달 감지 시 버튼 전송 중지
4. **정차 상태**: 일반 크루즈는 정차 시 자동 해제됨 (별도 처리 불필요)

---

## 테스트 방법

### 1. CAN 로그 확인 (벤치 테스트)
```bash
# panda로 CAN 메시지 모니터링
cd tools/cansim
python can_logger.py --bus 0 | grep "CLU11\|LVR12\|EMS16"
```

### 2. 리플레이 테스트
```bash
# 기존 로그로 비전 lead 동작 확인
tools/replay/replay <k7_log_file>
```

### 3. 실차 테스트 순서
1. 선행차 없이 정속 주행 테스트
2. 수동으로 앞차 접근 시 감속 확인
3. 앞차 가속 시 추종 확인
4. 컷인 상황 테스트

---

## 예상 제한사항

1. **정차 후 재출발**: 일반 크루즈는 정차 시 해제되므로 운전자가 재활성화 필요
2. **반응 속도**: 버튼 스팸 방식이므로 SCC 대비 반응이 느림 (약 0.5~1초 지연)
3. **급제동 한계**: 급제동 상황에서 운전자 브레이크 개입 필수
4. **최대 감속률**: 버튼 스팸으로 초당 약 10-14 kph 감속 가능 (2 kph × 5~7회)

---

## 구현 우선순위

1. **Phase 1**: interface.py + carstate.py 수정 (SCC 없음 인식)
2. **Phase 2**: scc_smoother.py 새 파일 생성 (버튼 제어 로직)
3. **Phase 3**: carcontroller.py 통합
4. **Phase 4**: 벤치 테스트 및 파라미터 튜닝
