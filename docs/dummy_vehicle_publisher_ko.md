# Dummy Vehicle Publisher 실행 방법

Orange Pi 벤치 환경에서 실제 차량이 연결된 것처럼 `pandaStates`, `carParams`, `carState`, `controlsState`, `radarState`를 publish해 UI와 calibration, 선행 차량 표시 흐름을 테스트하는 절차다.

## 전제

- 작업 디렉터리: `/home/orangepi/openpilot`
- SSH 계정: `orangepi@192.168.219.109`
- 암호: `orangepi`
- 카메라: `/dev/video0`
- 이 절차는 실차 주행용이 아니라 벤치 테스트용이다.

## 기존 프로세스 정리

```bash
cd /home/orangepi/openpilot

for n in manager.py _ui camerad _modeld; do
  pgrep -x "$n" || true
done | xargs -r kill -TERM

pgrep -f "^selfdrive.controls.controlsd" | xargs -r kill -TERM
pgrep -f "^selfdrive.controls.radard" | xargs -r kill -TERM
pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill -TERM

sleep 2

for n in manager.py _ui camerad _modeld; do
  pgrep -x "$n" || true
done | xargs -r kill -KILL

pgrep -f "^selfdrive.controls.controlsd" | xargs -r kill -KILL
pgrep -f "^selfdrive.controls.radard" | xargs -r kill -KILL
pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill -KILL
```

## Manager 실행: 벤치 dummy 테스트

`controlsd`는 `controlsState`, `carState`, `carParams`를 직접 publish하므로 dummy publisher와 동시에 실행하면 `MultiplePublishersError`가 난다. 이 문서의 dummy publisher는 `radarState`도 publish하므로 `radard`도 같이 막는다. 벤치 검증에서는 longitudinal MPC 생성물이 없어도 UI/modeld/calibration 흐름을 확인할 수 있도록 `plannerd`도 막는다.

`tools/run_orangepi_openpilot.sh`는 내부에서 `BLOCK`을 unset하므로 이 테스트에는 쓰지 않는다.

```bash
cd /home/orangepi/openpilot

setsid -f env \
  PATH=/tmp/openpilot-build-bin:$PATH \
  XDG_RUNTIME_DIR=/run/user/1000 \
  WAYLAND_DISPLAY=wayland-0 \
  QT_QPA_PLATFORM=wayland \
  DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
  PYTHONPATH=/home/orangepi/openpilot \
  OPENPILOT_BUILD_PLATFORM=linux_generic \
  PASSIVE=1 \
  USE_WEBCAM=1 \
  FORCE_IGNITION_ON=1 \
  NOSENSOR=1 \
  DISABLE_DRIVER_MONITORING=1 \
  NOBOARD=1 \
  BLOCK=controlsd,radard,plannerd \
  ROADCAM_FOURCC=MJPG \
  ROADCAM_WIDTH=1280 \
  ROADCAM_HEIGHT=720 \
  ROADCAM_FPS=20 \
  python3 selfdrive/manager/manager.py \
  >/tmp/openpilot-manager.log 2>&1 < /dev/null
```

카메라가 불안정하면 먼저 낮은 해상도로 테스트할 수 있다.

```bash
ROADCAM_WIDTH=640
ROADCAM_HEIGHT=360
ROADCAM_FPS=30
```

## Manager 실행: 실차

실차에서는 dummy publisher를 사용하지 않는다. `controlsd`, `radard`, `boardd`, `pandad`가 실제 차량/Panda/CAN 상태를 만들 수 있어야 하므로 `BLOCK`, `NOBOARD`, `FORCE_IGNITION_ON`을 넣지 않는다.

```bash
cd /home/orangepi/openpilot

pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill -TERM

setsid -f env \
  PATH=/tmp/openpilot-build-bin:$PATH \
  XDG_RUNTIME_DIR=/run/user/1000 \
  WAYLAND_DISPLAY=wayland-0 \
  QT_QPA_PLATFORM=wayland \
  DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
  PYTHONPATH=/home/orangepi/openpilot \
  OPENPILOT_BUILD_PLATFORM=linux_generic \
  PASSIVE=1 \
  USE_WEBCAM=1 \
  ROADCAM_FOURCC=MJPG \
  ROADCAM_WIDTH=1280 \
  ROADCAM_HEIGHT=720 \
  ROADCAM_FPS=20 \
  python3 selfdrive/manager/manager.py \
  >/tmp/openpilot-manager.log 2>&1 < /dev/null
```

실차에서 IMU/GPS 등 센서가 아직 준비되지 않은 bring-up 단계라면 임시로 `NOSENSOR=1`을 추가할 수 있다. 이 경우에도 `BLOCK`, `NOBOARD`, `FORCE_IGNITION_ON`은 넣지 않는다.

스크립트를 사용할 경우에는 실차 모드를 지정한다.

```bash
cd /home/orangepi/openpilot

pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill -TERM

ORANGEPI_REAL_CAR=1 ./tools/run_orangepi_openpilot.sh \
  >/tmp/openpilot-manager.log 2>&1 < /dev/null
```

## Dummy Publisher 실행

아래 publisher는 60 kph로 주행 중이고, 25 m 앞에 vision 기반 선행 차량이 있는 것처럼 publish한다. 또한 `liveCalibration`을 구독해서 실제 `controlsd` alert와 비슷하게 동작한다.

- `UNCALIBRATED`: `Calibration in Progress: xx%`
- `INVALID`: `Calibration Invalid`
- `CALIBRATED`: alert 없음

```bash
cd /home/orangepi/openpilot

cat >/tmp/dummy_vehicle_publisher.py <<'PY'
import os
import time

os.chdir('/home/orangepi/openpilot')
os.environ.setdefault('PYTHONPATH', '/home/orangepi/openpilot')

import cereal.messaging as messaging

CAL_UNCALIBRATED = 0
CAL_CALIBRATED = 1
speed_ms = 60.0 / 3.6
lead_d_rel = 25.0
lead_v_rel = -1.5
lead_y_rel = 0.0

pm = messaging.PubMaster(['controlsState', 'carState', 'carParams', 'pandaStates', 'radarState'])
sm = messaging.SubMaster(['liveCalibration'])
frame = 0
print('dummy_vehicle_publisher: started with radarState lead dRel=25m speed_kph=60', flush=True)

while True:
  sm.update(0)
  lc = sm['liveCalibration']
  cal_status = int(lc.calStatus)
  cal_perc = int(lc.calPerc)

  controls = messaging.new_message('controlsState')
  controls.valid = True
  cs = controls.controlsState
  cs.enabled = False
  cs.active = False
  cs.engageable = True
  cs.state = 'disabled'
  cs.alertStatus = 'normal'
  cs.alertSound = 'none'
  cs.vCruise = 60.0
  cs.lateralControlMethod = 0
  cs.steerRatio = 13.0
  cs.longControlState = 'off'

  if cal_status == CAL_UNCALIBRATED:
    cs.alertSize = 'mid'
    cs.alertText1 = f'Calibration in Progress: {cal_perc}%'
    cs.alertText2 = 'Drive Above 24 km/h'
    cs.alertType = 'calibrationIncomplete/permanent'
  elif cal_status != CAL_CALIBRATED:
    cs.alertSize = 'mid'
    cs.alertText1 = 'Calibration Invalid'
    cs.alertText2 = 'Remount Device & Recalibrate'
    cs.alertType = 'calibrationInvalid/permanent'
  else:
    cs.alertSize = 'none'
    cs.alertText1 = ''
    cs.alertText2 = ''
    cs.alertType = ''
  pm.send('controlsState', controls)

  car_state = messaging.new_message('carState')
  car_state.valid = True
  car_state.carState.vEgo = speed_ms
  car_state.carState.vEgoRaw = speed_ms
  car_state.carState.aEgo = 0.0
  car_state.carState.standstill = False
  car_state.carState.standStill = False
  car_state.carState.gearShifter = 'drive'
  car_state.carState.canValid = True
  car_state.carState.cruiseState.available = True
  car_state.carState.cruiseState.enabled = False
  car_state.carState.cruiseState.standstill = False
  car_state.carState.cruiseState.speed = speed_ms
  car_state.carState.cruiseGapSet = 3
  pm.send('carState', car_state)

  car_params = messaging.new_message('carParams')
  car_params.valid = True
  cp = car_params.carParams
  cp.carName = 'hyundai'
  cp.carFingerprint = 'DUMMY HYUNDAI'
  cp.dashcamOnly = False
  cp.openpilotLongitudinalControl = False
  cp.pcmCruise = True
  cp.steerControlType = 'torque'
  cp.transmissionType = 'automatic'
  cp.mass = 1600.0
  cp.wheelbase = 2.8
  cp.centerToFront = 1.1
  cp.steerRatio = 13.0
  pm.send('carParams', car_params)

  panda_states = messaging.new_message('pandaStates', 1)
  panda_states.valid = True
  ps = panda_states.pandaStates[0]
  ps.pandaType = 'blackPanda'
  ps.ignitionLine = True
  ps.ignitionCan = True
  ps.controlsAllowed = False
  pm.send('pandaStates', panda_states)

  radar = messaging.new_message('radarState')
  radar.valid = True
  rs = radar.radarState
  rs.mdMonoTime = 0
  rs.carStateMonoTime = 0
  rs.radarErrors = []
  lead = rs.leadOne
  lead.status = True
  lead.dRel = lead_d_rel
  lead.yRel = lead_y_rel
  lead.vRel = lead_v_rel
  lead.aRel = 0.0
  lead.vLead = speed_ms + lead_v_rel
  lead.vLeadK = speed_ms + lead_v_rel
  lead.aLeadK = 0.0
  lead.aLeadTau = 1.5
  lead.fcw = False
  lead.modelProb = 0.95
  lead.radar = False
  rs.leadTwo.status = False
  pm.send('radarState', radar)

  frame += 1
  if frame % 100 == 0:
    print(
      f'dummy_vehicle_publisher: speed_kph=60 lead_status=True dRel={lead_d_rel} '
      f'vRel={lead_v_rel} calStatus={cal_status} '
      f'calPerc={cal_perc} alert={cs.alertText1!r}',
      flush=True,
    )
  time.sleep(0.02)
PY

setsid -f env PYTHONPATH=/home/orangepi/openpilot \
  python3 /tmp/dummy_vehicle_publisher.py dummy_vehicle_publisher \
  >/tmp/dummy_vehicle_publisher.log 2>&1 < /dev/null
```

## 상태 확인

```bash
pgrep -af "python3 selfdrive/manager/manager.py|[.]_ui|[c]amerad|[.]_modeld|^selfdrive.controls.controlsd|^selfdrive.controls.radard|[d]ummy_vehicle_publisher" || true

tail -20 /tmp/dummy_vehicle_publisher.log
tail -80 /tmp/openpilot-manager.log
```

메시지 값 확인:

```bash
cd /home/orangepi/openpilot

PYTHONPATH=/home/orangepi/openpilot python3 - <<'PY'
import time
import cereal.messaging as messaging

services = ['liveCalibration', 'controlsState', 'carState', 'radarState', 'roadCameraState', 'modelV2', 'deviceState']
sm = messaging.SubMaster(services)
seen = {s: 0 for s in services}
end = time.monotonic() + 3

while time.monotonic() < end:
  sm.update(100)
  for s in services:
    if sm.updated[s]:
      seen[s] += 1

lc = sm['liveCalibration']
cs = sm['controlsState']
car = sm['carState']
lead = sm['radarState'].leadOne

print('seen', seen)
print('started', sm['deviceState'].started)
print('calStatus', lc.calStatus, 'calPerc', lc.calPerc, 'validBlocks', lc.validBlocks)
print('alertSize', cs.alertSize, 'alertText1', repr(cs.alertText1), 'alertText2', repr(cs.alertText2))
print('vEgo_kph', car.vEgo * 3.6, 'standstill', car.standstill)
print('leadOne', lead.status, 'dRel', lead.dRel, 'vRel', lead.vRel, 'radar', lead.radar)
print('road_frame_id', sm['roadCameraState'].frameId)
PY
```

정상 기준:

- `controlsd` 프로세스가 없어야 한다.
- `radard` 프로세스가 없어야 한다.
- `dummy_vehicle_publisher`가 있어야 한다.
- `vEgo_kph`가 약 `60`이어야 한다.
- `leadOne status=True`, `dRel=25` 근처여야 한다.
- `deviceState.started=True`여야 한다.
- `roadCameraState`, `modelV2`가 증가해야 한다.

## 중지

Dummy publisher만 중지:

```bash
pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill
```

테스트 manager까지 중지:

```bash
for n in manager.py _ui camerad _modeld; do
  pgrep -x "$n" || true
done | xargs -r kill
```

## 실제 controlsd와 다른 점

실제 차량에서는 `controlsd`가 CAN에서 `carState`를 만들고 calibration event를 `controlsState` alert로 publish한다. 이 벤치 환경에서는 실제 CAN이 없고 이 fork의 `controlsd`가 `carState`도 직접 publish하므로, dummy publisher와 동시에 켜면 publisher 충돌이 난다.

또한 실제 차량에서는 `radard`가 `modelV2.leadsV3`의 vision lead를 `radarState`로 변환해서 선행 차량 삼각형을 띄운다. 이 벤치 환경에서는 `radard`가 정상 publish하지 못할 수 있으므로 dummy publisher가 `radarState`도 직접 publish한다.

그래서 이 문서의 dummy publisher는 `controlsd`, `radard` 대신 `controlsState`, `radarState`까지 publish한다. 단, calibration alert 문구는 `liveCalibration` 상태를 보고 실제 `controlsd`의 조건과 비슷하게 만든다.
