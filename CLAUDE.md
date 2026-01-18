# CLAUDE.md

이 파일은 Claude Code (claude.ai/code)가 이 저장소의 코드 작업 시 참고하는 가이드입니다.

## 프로젝트 개요

이것은 **OPKR (OpenPilot Korea)**, comma.ai의 openpilot을 기반으로 현대, 기아, 제네시스 차량에 특화된 커뮤니티 포크입니다. 어댑티브 크루즈 컨트롤(ACC), 자동 차선 유지(ALC), 전방 충돌 경고(FCW), 차선 이탈 경고(LDW) 기능을 제공하는 오픈소스 운전자 보조 시스템입니다.

- **메인 저장소**: https://github.com/openpilotkr/openpilot
- **안정 브랜치**: `OPKR`
- **개발 브랜치**: `OPKR_test`
- **PR용 업스트림 브랜치**: `opkr_c2`
- **커뮤니티**: https://discord.gg/pppFp2pVW3

## 타겟 차량: 기아 K7 (YG, 2016-2019)

이 프로젝트는 **기아 K7 (YG 세대)** 를 기준으로 작업합니다.

### K7 차량 정보

| 항목 | 값 |
|------|-----|
| 모델 코드 | `CAR.K7_YG` (일반) / `CAR.K7_HEV_YG` (하이브리드) |
| 연식 | 2016-2019 |
| 하네스 타입 | `Harness.hyundai_c` |
| DBC 파일 | `hyundai_kia_generic` |
| 레이더 DBC | `hyundai_kia_mando_front_radar` |
| 기어 인식 | 클러스터 기어 사용 (`use_cluster_gears`) |

### K7 CAN Fingerprint

K7은 다음 CAN ID 패턴으로 식별됩니다 ([values.py:396-402](selfdrive/car/hyundai/values.py#L396-L402)):

```python
CAR.K7_YG: [{
  67: 8, 68: 8, 127: 8, 304: 8, 320: 8, 339: 8, 356: 4, 544: 8, 546: 8,
  593: 8, 608: 8, 688: 5, 809: 8, 832: 8, 854: 7, 870: 7, 871: 8, 872: 8,
  897: 8, 902: 8, 903: 8, 916: 8, 1040: 8, 1056: 8, 1057: 8, 1078: 4,
  1107: 5, 1136: 8, 1151: 6, 1156: 8, 1162: 4, 1168: 7, 1170: 8, 1173: 8,
  1265: 4, 1280: 1, 1287: 4, 1290: 8, 1292: 8, 1294: 8, 1312: 8, 1322: 8,
  1345: 8, 1348: 8, 1363: 8, 1369: 8, 1378: 4, 1384: 8, 1407: 8, 1419: 8,
  1427: 6, 1444: 8, 1456: 4, 1470: 8
}]
```

### K7 주요 CAN ID

| CAN ID | 용도 | 설명 |
|--------|------|------|
| 593 | MDPS (모터 조향) | mdpsBus 감지용 |
| 688 | SAS (스티어링 각도) | sasBus 감지용 |
| 1056 | SCC (스마트 크루즈) | sccBus 감지용 |
| 608, 809 | EMS (엔진 관리) | 속도/토크 정보 |
| 871 | LVR | lvrAvailable 감지 |
| 1419 | BSM (후측방 경고) | bsmAvailable 감지 |

## 빌드 시스템 및 명령어

### 개발 환경 설정

```bash
# 초기 설정
git submodule update --init

# Ubuntu 설정
tools/ubuntu_setup.sh

# macOS 설정
tools/mac_setup.sh

# Python 환경 활성화
cd openpilot && pipenv shell
```

### 빌드 명령어 (SCons 기반)

```bash
# 전체 빌드
scons -u -j$(nproc)

# 테스트 포함 빌드
scons -u -j$(nproc) --test

# 새니타이저 포함 빌드
scons -u -j$(nproc) --asan        # AddressSanitizer
scons -u -j$(nproc) --ubsan       # UndefinedBehaviorSanitizer

# 컴파일 데이터베이스 생성
scons -u -j$(nproc) --compile_db

# 클린 빌드
scons -u -j$(nproc) --clean

# 특정 컴포넌트만 빌드
scons -u -j$(nproc) selfdrive/
```

### 테스트

```bash
# 통합 테스트
python selfdrive/test/test_onroad.py
python selfdrive/test/test_msg.py

# 차량 인터페이스 테스트
python -m pytest selfdrive/car/tests/
python selfdrive/car/tests/test_car_interfaces.py
python selfdrive/car/tests/test_fingerprints.py

# 특정 테스트 파일 실행
python -m pytest selfdrive/car/tests/test_car_interfaces.py -v

# 특정 테스트 실행
python -m pytest selfdrive/car/tests/test_car_interfaces.py::TestCarInterfaces::test_car_interfaces -v

# 코드 품질 검사
pre-commit run --all
mypy selfdrive/
pylint selfdrive/
```

### 로컬 실행

```bash
# 전체 실행
./launch_openpilot.sh

# 매니저만 실행 (모든 프로세스 시작)
python selfdrive/manager/manager.py

# 디버그 모드
export DEBUG=1
python selfdrive/manager/manager.py
```

## 아키텍처 개요

### 고수준 시스템 설계

openpilot은 **데몬 기반 프로세스 아키텍처**를 사용하며, 독립된 프로세스들이 **Cap'n Proto 메시지**를 통해 ZMQ/msgq로 통신합니다. 시스템은 100Hz 제어 루프를 실행하여 차량 상태를 읽고, 센서 데이터를 처리하고, 플래닝 알고리즘을 실행하고, CAN 버스를 통해 차량에 제어 명령을 전송합니다.

**핵심 아키텍처 결정사항:**
- 결정론적 동작을 위한 100Hz 동기 제어 루프
- 격리 및 독립적 재시작을 위한 프로세스별 모듈
- 성능을 위한 Cap'n Proto 제로 복사 IPC
- 중요 프로세스를 위한 실시간 스케줄링 (RT-FIFO)
- 리플레이 기반 개발 및 테스트를 위한 전체 데이터 로깅

### 핵심 디렉토리 구조

```
openpilot/
├── cereal/              # 메시징 시스템 (Cap'n Proto 스키마)
│   ├── car.capnp        # 차량 제어 메시지 정의
│   ├── log.capnp        # 로깅 메시지 정의 (90+ 타입)
│   └── services.py      # 서비스 레지스트리 (~40개 서비스)
├── common/              # 공유 라이브러리
│   ├── params.py        # 파라미터 관리 (/data/params/d/)
│   └── conversions.py   # 단위 변환 (CV.MPH_TO_MS 등)
├── opendbc/             # 차량용 CAN 데이터베이스 정의
├── panda/               # 차량 통신 하드웨어 인터페이스
├── selfdrive/           # 핵심 드라이빙 로직
│   ├── car/             # 브랜드별 차량 구현
│   │   └── hyundai/     # 현대/기아/제네시스 (K7 포함)
│   ├── controls/        # 플래닝 및 제어 (controlsd, plannerd, radard)
│   ├── manager/         # 프로세스 관리 및 조정
│   ├── ui/              # Qt 기반 사용자 인터페이스
│   │   └── qt/widgets/opkr.cc  # OPKR 특화 UI 위젯
│   ├── modeld/          # 신경망 모델 러너
│   ├── camerad/         # 카메라 센서 인터페이스
│   ├── locationd/       # GPS 및 정위치 (EKF)
│   ├── boardd/          # CAN 버스 통신 (panda 인터페이스)
│   ├── navi/            # 외부 네비게이션 지원
│   └── test/            # 통합 테스트
├── third_party/         # 외부 의존성 (ACADOS, SNPE, libyuv)
└── tools/               # 개발 도구 (replay, simulator)
```

### 프로세스 아키텍처

시스템은 `selfdrive/manager/manager.py`가 모든 데몬을 시작하고 모니터링합니다.

**하드웨어 프로세스 (C/C++):**
- `camerad` - 카메라 캡처 (30fps 도로, 20fps 운전자 모니터링)
- `modeld` - 신경망 추론 (supercombo 비전 모델)
- `sensord` - IMU/센서 읽기
- `boardd` - panda 하드웨어를 통한 CAN 통신
- `loggerd` - 데이터 로깅
- `ui` - Qt 기반 사용자 인터페이스
- `locationd` - 확장 칼만 필터를 사용한 GPS/정위치
- `ubloxd` - GPS 수신기 인터페이스

**제어 프로세스 (Python):**
- `controlsd` - 메인 100Hz 제어 루프 (모든 것을 조정)
- `plannerd` - 모션 플래닝 (경로, 속도 목표)
- `radard` - 레이더/선행차 추적
- `calibrationd` - 카메라 캘리브레이션
- `dmonitoringd` - 운전자 모니터링 ML 모델
- `thermald` - 열 관리
- `pandad` - Panda 통신 래퍼

**프로세스 설정:** [process_config.py](selfdrive/manager/process_config.py)에서 활성화 조건, 지속성 레벨, CPU 워치독 제약을 정의합니다.

### 제어 흐름 (100Hz 메인 루프)

시스템의 핵심은 [controlsd.py](selfdrive/controls/controlsd.py)입니다:

```
controlsd.py (100Hz)
├── 입력 읽기:
│   ├── carState (차량 인터페이스에서 차량 상태)
│   ├── sensorEvents (IMU 데이터)
│   ├── radarState (선행차 추적)
│   ├── modelData (비전 모델 출력)
│   ├── liveLocationKalman (정위치)
│   └── 사용자 입력 (크루즈 버튼, 스티어링)
├── 플래닝:
│   ├── plannerd - 모션 플래닝 (ACC 목표 속도, 횡방향 경로)
│   └── radard - 레이더 처리 (선행차 추적)
├── 횡방향 제어 (스티어링):
│   ├── PID 컨트롤러
│   ├── INDI 컨트롤러
│   ├── LQR 컨트롤러
│   └── Torque 기반 컨트롤러
├── 종방향 제어 (가속):
│   ├── Longitudinal MPC
│   ├── ACC 로직
│   └── 안전 제한
└── 출력: carControl 메시지 → boardd → panda → 차량 CAN
```

## 현대/기아 차량 인터페이스 (K7 중심)

### 파일 구조

```
selfdrive/car/hyundai/
├── interface.py       # CarInterface 구현 - 차량 핑거프린팅, 기능 감지
├── carstate.py        # CarState - CAN 메시지를 파싱하여 차량 상태 추출
├── carcontroller.py   # CarController - CAN 명령을 차량에 전송
├── hyundaican.py      # CAN 메시지 포맷팅 함수
├── values.py          # 차종 정의 및 설정 (K7 포함)
├── tunes.py           # 제어 튜닝 설정 (PID/INDI/LQR/Torque)
├── radar_interface.py # 레이더 인터페이스
└── navicontrol.py     # 네비게이션 제어
```

### 차량 인터페이스 초기화 ([interface.py](selfdrive/car/hyundai/interface.py))

K7 연결 시 자동으로 감지되는 버스 및 기능:

```python
# 버스 자동 감지 (라인 39-48)
ret.mdpsBus = 1 if 593 in fingerprint[1] else 0      # MDPS (전동 조향)
ret.sasBus = 1 if 688 in fingerprint[1] else 0       # SAS (스티어링 각도)
ret.sccBus = 0 if 1056 in fingerprint[0] else ...    # SCC (스마트 크루즈)
ret.fcaBus = 0 if 909 in fingerprint[0] else ...     # FCA (전방 충돌 방지)
ret.bsmAvailable = True if 1419 in fingerprint[0]    # BSM (후측방 경고)
ret.lfaAvailable = True if 1157 in fingerprint[2]    # LFA (차선 유지 보조)
ret.lvrAvailable = True if 871 in fingerprint[0]     # LVR
ret.emsAvailable = True if 608 and 809 in fingerprint[0]  # EMS (엔진 관리)
```

### 횡방향 제어 방법 ([tunes.py](selfdrive/car/hyundai/tunes.py))

OPKR은 4가지 횡방향 제어 방법을 지원합니다:

| 방법 | 설명 | 파라미터 |
|------|------|----------|
| **PID** | 기본 PID 제어 | `PidKp`, `PidKi`, `PidKd`, `PidKf` |
| **INDI** | 역학 기반 제어 | `InnerLoopGain`, `OuterLoopGain`, `TimeConstant`, `ActuatorEffectiveness` |
| **LQR** | 최적 제어 | `Scale`, `LqrKi`, `DcGain` |
| **Torque** | 토크 기반 제어 | `TorqueKp`, `TorqueKf`, `TorqueKi`, `TorqueFriction`, `TorqueMaxLatAccel` |
| **ATOM** | 통합 제어 (Torque+LQR+INDI+PID) | 위 모든 파라미터 |

### 제어 파라미터 ([values.py:12-22](selfdrive/car/hyundai/values.py#L12-L22))

```python
class CarControllerParams:
  ACCEL_MIN = -4.0  # m/s² (최대 감속)
  ACCEL_MAX = 2.0   # m/s² (최대 가속)

  # UI에서 조정 가능한 파라미터
  STEER_MAX = int(Params().get("SteerMaxAdj"))           # 기본값 384
  STEER_DELTA_UP = int(Params().get("SteerDeltaUpAdj"))  # 기본값 3
  STEER_DELTA_DOWN = int(Params().get("SteerDeltaDownAdj"))  # 기본값 7
```

## OPKR 특화 기능

### UI 기능

- 대부분의 파라미터에 대한 고급 온스크린 설정
- UI를 통한 실시간 튜닝 (횡방향/종방향 제어)
- 다중 크루즈 모드 (Stock, Dist+Curv, Dist only, Curv only)
- MapBox 네비게이션 통합
- OSM 통합 (속도 제한 및 곡선 감속)
- 네트워크 정보 표시 (IP, SSID, 캐리어, 신호 강도)
- 다중 드라이버 프리셋

### HKG(현대/기아/제네시스) 특화 기능

- **SmartMDPS** 지원 (0km/h까지 조향 가능)
- **SCC 버튼 스팸** (속도 조절)
- CAN2에서 **SCC 버스 자동 인식** (종방향 제어용)
- 다중 횡방향 제어 옵션 (PID/INDI/LQR/Torque)
- 자동 차선 모드 선택 (laneless vs lanefull)
- UI에서 광범위한 튜닝 파라미터 노출

### 주요 OPKR 파라미터

런타임 파라미터는 `/data/params/d/`에 저장되며 `common.params.Params` 클래스로 접근합니다.

**횡방향 제어:**
- `LateralControlMethod` - 제어 방법 (0=PID, 1=INDI, 2=LQR, 3=Torque, 4=ATOM)
- `SteerMaxAdj`, `SteerDeltaUpAdj`, `SteerDeltaDownAdj` - 스티어링 제한
- `SteerRatioAdj` - 스티어링 비율
- `TireStiffnessFactorAdj` - 타이어 강성

**종방향 제어:**
- `RadarDisable` - 레이더 비활성화 (openpilot 종방향 제어)
- `CruiseStatemodeSelInit` - 크루즈 상태 모드

**UI/시스템:**
- `OpkrEnableLogger` - 로깅 활성화
- `OpkrEnableUploader` - 업로드 활성화
- `OpkrAutoShutdown` - 자동 종료
- `OpkrUIBrightness` - UI 밝기
- `OPKRNaviSelect` - 네비 선택 (0=내장, 1=Google, 2=Naver, 3=Kakao, 4=Waze, 5=iNavi)

## 주요 파일 및 진입점

| 파일 | 용도 |
|------|------|
| [launch_openpilot.sh](launch_openpilot.sh) | 포크 설치 진입점 |
| [launch_chffrplus.sh](launch_chffrplus.sh) | 장치 부팅 스크립트 |
| [selfdrive/manager/manager.py](selfdrive/manager/manager.py) | 프로세스 코디네이터 및 시작 관리자 |
| [selfdrive/manager/process_config.py](selfdrive/manager/process_config.py) | 프로세스 정의 및 활성화 조건 |
| [selfdrive/manager/process.py](selfdrive/manager/process.py) | 프로세스 실행 (Native/Python) |
| [selfdrive/controls/controlsd.py](selfdrive/controls/controlsd.py) | 메인 100Hz 제어 루프 |
| [selfdrive/controls/plannerd.py](selfdrive/controls/plannerd.py) | 모션 플래닝 |
| [selfdrive/controls/radard.py](selfdrive/controls/radard.py) | 레이더 처리 |
| [selfdrive/car/hyundai/interface.py](selfdrive/car/hyundai/interface.py) | 차량 핑거프린팅, 기능 감지 |
| [selfdrive/car/hyundai/carstate.py](selfdrive/car/hyundai/carstate.py) | CAN 메시지 파싱 |
| [selfdrive/car/hyundai/carcontroller.py](selfdrive/car/hyundai/carcontroller.py) | CAN 명령 전송 |
| [selfdrive/car/hyundai/values.py](selfdrive/car/hyundai/values.py) | 차종 정의 (K7 포함) |
| [selfdrive/car/hyundai/tunes.py](selfdrive/car/hyundai/tunes.py) | 제어 튜닝 설정 |
| [selfdrive/car/car_helpers.py](selfdrive/car/car_helpers.py) | 차량 핑거프린팅 로직 |
| [SConstruct](SConstruct) | 빌드 시스템 설정 |
| [cereal/services.py](cereal/services.py) | 서비스 정의 및 포트 |

## 개발 가이드라인

### 새 차량 추가하기

1. `opendbc/`에 CAN 정의 추가
2. `selfdrive/car/{brand}/` 디렉토리 생성 및 인터페이스 파일 작성
3. `CarInterface`, `CarState`, `CarController`, `RadarInterface` 구현
4. 핑거프린팅 로직 추가 (DBC + CAN 메시지 패턴)
5. Jenkins CI에서 하드웨어 인더루프 테스트

### 코드 조직

**C++ 네이티브 프로세스:**
- 독립 바이너리로 컴파일 (Python 모듈 아님)
- 공유 유틸리티는 `selfdrive/common/` 사용
- 중요 프로세스는 실시간 스케줄링 (RT-FIFO 우선순위)
- `SConstruct`에서 빌드 설정

**Python 프로세스:**
- IPC에 cereal 메시징 사용
- `selfdrive/{module}/` 구조 따름
- 공통 베이스 클래스 상속 (예: `CarInterfaceBase`)

### 테스트 전략

**CI/CD 파이프라인:**
- GitHub Actions: 린팅, 유닛 테스트, 정적 분석
- Jenkins: EON/TICI 장치에서 하드웨어 인더루프 테스트
- Pre-commit 훅: 자동 코드 품질 검사

**리플레이 기반 개발:**
- 주행 중 모든 메시지 로깅
- `tools/replay/`로 디버깅용 로그 리플레이
- 전체 드라이브의 결정론적 리플레이

## 중요 참고사항

- 이것은 중요한 안전 관련 알파 품질 연구 소프트웨어입니다
- 변경사항은 항상 안전한 환경에서 테스트하세요 (시뮬레이터, 리플레이, 통제된 도로 주행)
- 100Hz 제어 루프 타이밍이 중요합니다 - 블로킹 작업을 피하세요
- CAN 메시지 타이밍 및 안전 모델은 panda 펌웨어가 강제합니다
- 모든 프로세스는 메시지 손실을 우아하게 처리해야 합니다 (프로세스 재시작 가능)
- 하드웨어 타겟은 EON/TICI (ARM64)이지만 PC 개발도 지원됩니다

## 문서 리소스

- `/docs/CONTRIBUTING.md` - 기여 가이드라인
- `/docs/CARS.md` - 지원 차량
- `/tools/README.md` - 개발 도구 문서
- `/tools/sim/README.md` - CARLA 시뮬레이터 설정
- `/tools/replay/README.md` - 로그 리플레이 도구
- https://docs.comma.ai/ - comma.ai 공식 문서
- https://blog.comma.ai/ - 기술 딥다이브
