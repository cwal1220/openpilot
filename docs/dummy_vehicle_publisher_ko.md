# Dummy Vehicle Publisher 실행 방법

Orange Pi 벤치 환경에서 실제 차량이 연결된 것처럼 `pandaStates`, `carParams`, `carState`, `controlsState`를 publish해 UI와 calibration 흐름을 테스트하는 절차다.

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

`controlsd`는 `controlsState`, `carState`, `carParams`를 직접 publish하므로 dummy publisher와 동시에 실행하면 `MultiplePublishersError`가 난다. 이 문서의 기본 목적은 calibration/UI/modeld 확인이므로 `radard`, `plannerd`도 같이 막아 벤치 환경을 단순하게 둔다.

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

아래 publisher는 60 kph로 주행 중인 것처럼 `controlsState`, `carState`, `carParams`, `pandaStates`를 publish한다. 또한 `liveCalibration`을 구독해서 실제 `controlsd` alert와 비슷하게 동작한다.

- `UNCALIBRATED`: `Calibration in Progress: xx%`
- `INVALID`: `Calibration Invalid`
- `CALIBRATED`: alert 없음

```bash
cd /home/orangepi/openpilot

setsid -f env PYTHONPATH=/home/orangepi/openpilot \
  python3 tools/dummy_vehicle_publisher.py \
  >/tmp/dummy_vehicle_publisher.log 2>&1 < /dev/null
```

속도를 바꾸고 싶으면 `--speed-kph`를 사용한다.

```bash
setsid -f env PYTHONPATH=/home/orangepi/openpilot \
  python3 tools/dummy_vehicle_publisher.py --speed-kph 80 \
  >/tmp/dummy_vehicle_publisher.log 2>&1 < /dev/null
```

## 상태 확인

```bash
pgrep -af "python3 selfdrive/manager/manager.py|[.]_ui|[c]amerad|[.]_modeld|^selfdrive.controls.controlsd|[d]ummy_vehicle_publisher" || true

tail -20 /tmp/dummy_vehicle_publisher.log
tail -80 /tmp/openpilot-manager.log
```

메시지 값 확인:

```bash
cd /home/orangepi/openpilot

PYTHONPATH=/home/orangepi/openpilot python3 - <<'PY'
import time
import cereal.messaging as messaging

services = ['liveCalibration', 'controlsState', 'carState', 'roadCameraState', 'modelV2', 'deviceState']
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

print('seen', seen)
print('started', sm['deviceState'].started)
print('calStatus', lc.calStatus, 'calPerc', lc.calPerc, 'validBlocks', lc.validBlocks)
print('alertSize', cs.alertSize, 'alertText1', repr(cs.alertText1), 'alertText2', repr(cs.alertText2))
print('vEgo_kph', car.vEgo * 3.6, 'standstill', car.standstill)
print('road_frame_id', sm['roadCameraState'].frameId)
PY
```

정상 기준:

- `controlsd` 프로세스가 없어야 한다.
- `dummy_vehicle_publisher`가 있어야 한다.
- `vEgo_kph`가 약 `60`이어야 한다.
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

그래서 이 문서의 dummy publisher는 `controlsd` 대신 `controlsState`, `carState`, `carParams`, `pandaStates`를 publish한다. 단, calibration alert 문구는 `liveCalibration` 상태를 보고 실제 `controlsd`의 조건과 비슷하게 만든다.

이 publisher는 `radarState`를 publish하지 않는다. 선행 차량 삼각형 확인이 필요하면 `radard`를 별도로 정상 동작시키거나, 별도 radar dummy publisher를 사용한다.
