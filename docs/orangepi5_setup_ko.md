# Orange Pi 5 설치 메모

이 문서는 Orange Pi 5에서 openpilot generic Linux ARM64 포팅 작업을 다시 이어갈 때,
누락된 패키지를 빠르게 재설치하기 위한 메모다.

## 1. 권장 설치 방법

저장소 루트에서 아래 명령을 실행한다.

```bash
cd ~/openpilot
tools/orangepi_setup.sh
```

한글 UI 폰트까지 같이 설치하려면 아래처럼 실행한다.

```bash
cd ~/openpilot
tools/orangepi_setup.sh --with-korean-fonts
```

스크립트는 Panda USB 접근을 위한 udev rule도 같이 설치한다.

## 2. 스크립트가 설치하는 항목

APT 패키지:

- `scons`
- `clang`, `cmake`, `build-essential`
- `gcc-arm-none-eabi`, `binutils-arm-none-eabi`
- `capnproto`, `libcapnp-dev`
- `libusb-1.0-0-dev`
- `libzmq3-dev`
- `libjpeg-dev`
- `libopencv-dev`
- `libyuv-dev`
- `libeigen3-dev`
- `libegl1-mesa-dev`, `libgles2-mesa-dev`
- `opencl-headers`, `ocl-icd-opencl-dev`, `mesa-opencl-icd`, `pocl-opencl-icd`, `clinfo`
- `python3-dev`, `python3-pip`, `python3-numpy`, `python3-jinja2`, `cython3`

Python 패키지:

- `sentry-sdk`
- `pycapnp`
- `Cython==0.29.37`
- `onnx`
- `onnxruntime`
- `pyzmq`
- `libusb1`
- `pyserial`
- `setproctitle`
- `tqdm`
- `crcmod`
- `casadi`
- `cffi`
- `sympy`
- `smbus2`

옵션 패키지:

- `fonts-nanum`
- `fonts-noto-cjk`

## 3. 설치 후 바로 확인할 것

```bash
clinfo | sed -n '1,40p'
python3 -c 'import capnp, zmq, usb1, serial, sentry_sdk, tqdm, crcmod, cffi, onnx'
```

카메라/모델 기본 빌드 확인:

```bash
cd ~/openpilot
OPENPILOT_BUILD_PLATFORM=linux_generic USE_WEBCAM=1 NOSENSOR=1 \
  scons -j$(nproc) selfdrive/camerad/camerad selfdrive/modeld/_modeld
```

Panda/boardd 기본 빌드 확인:

```bash
cd ~/openpilot
OPENPILOT_BUILD_PLATFORM=linux_generic \
  scons -j$(nproc) selfdrive/boardd/boardd panda/board/obj/panda.bin.signed panda/board/obj/panda_h7.bin.signed
```

## 4. 현재까지 확인된 추가 이슈

- `selfdrive/hardware/__init__.py`는 generic Linux ARM64가 Android 의존성을 미리 import하지 않도록 지연 import로 보정했다.
- `selfdrive/controls/lib/*_mpc_lib`의 `acados` 생성 산출물은 처음 한 번 별도 빌드가 필요할 수 있다.
- `selfdrive/modeld/runners/onnx_runner.py`는 실행권한이 필요하므로 bootstrap 스크립트에서 `chmod +x`를 같이 수행한다.
- 현재 Orange Pi 5 + UVC webcam 조합에서는 `ROADCAM_FOURCC=MJPG`가 가장 안정적으로 확인됐다.
- Panda는 USB로만 연결해도 `plugdev` udev rule이 없으면 `lsusb`에는 보여도 `Panda.list()`가 빈 배열로 나올 수 있다.
- Panda가 bootstub 상태인 경우에는 `panda/board/obj/panda*.bin.signed` 산출물이 있어야 자동 복구가 가능하다.

## 5. 실행 명령 예시

### 벤치 bring-up

Panda/CAN 없이 카메라, modeld, UI 중심으로 확인할 때는 `NOBOARD=1`, `FORCE_IGNITION_ON=1`, `NOSENSOR=1`을 사용한다. dummy publisher를 같이 쓸 때는 `controlsd`, `radard`와 publisher 충돌을 피하고 longitudinal MPC 생성물 없이 벤치 검증이 가능하도록 `BLOCK=controlsd,radard,plannerd`를 지정한다.

```bash
cd ~/openpilot

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

### 실차 실행

실차에서는 dummy publisher를 먼저 끄고, `BLOCK`, `NOBOARD`, `FORCE_IGNITION_ON` 없이 manager를 실행한다. Panda/CAN 기반으로 `controlsd`, `radard`, `boardd`, `pandad`가 직접 상태를 만들어야 한다.

```bash
cd ~/openpilot

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

실차에서 IMU/GPS 등 센서 bring-up이 아직 끝나지 않았으면 임시로 `NOSENSOR=1`을 추가할 수 있다. 그래도 실차 CAN/Panda 확인 목적이라면 `NOBOARD=1`은 넣지 않는다.

동일한 실차 실행은 스크립트로도 가능하다.

```bash
cd ~/openpilot

pgrep -af "[d]ummy_vehicle_publisher" | cut -d " " -f1 | xargs -r kill -TERM

ORANGEPI_REAL_CAR=1 ./tools/run_orangepi_openpilot.sh \
  >/tmp/openpilot-manager.log 2>&1 < /dev/null
```

## 6. 2026-04-17 기준 검증 상태

- Orange Pi 5에서 `pocl-opencl-icd` 기반 CPU OpenCL device 인식 확인
- `selfdrive/camerad/camerad`, `selfdrive/modeld/_modeld`, `selfdrive/locationd/locationd`, `selfdrive/clocksd/clocksd` 빌드 확인
- `USE_WEBCAM=1`, `FORCE_IGNITION_ON=1`, `ROADCAM_FOURCC=MJPG` 조건에서 `manager` onroad 경로 기동 확인
- `camerad` 첫 프레임 수신, 첫 `VisionIPC send` 확인
- `modeld`에서 `modelV2`, `cameraOdometry`가 실제로 발행되는 것까지 확인
- 보드 재부팅 직후에도 같은 조건으로 재실행했을 때 동일한 onroad bring-up이 재현됨
- Panda USB 권한과 펌웨어 산출물을 갖춘 뒤 `pandad -> boardd`까지 실제 연결 확인

메시지 구독 기준 실제 관측값:

- 약 8초 동안 `roadCameraState` 200회 수신
- 같은 구간 `modelV2` 94회 수신
- 같은 구간 `cameraOdometry` 94회 수신
- 재부팅 직후 재검증에서도 약 6초 동안 `roadCameraState` 151회, `modelV2` 71회, `cameraOdometry` 71회 수신
- Panda 복구 후 `pandad`가 `boardd`를 실행하고 `connected to board` 로그까지 확인
- 같은 bench 조건에서 `pandaStates` 11회, `peripheralState` 11회, `modelV2` 66회 수신 확인
- 당시 Panda 상태: `blackPanda`, `ignitionLine=False`, `ignitionCan=False`, `harnessStatus=flipped`, `safetyModel=noOutput`

즉 현재 Orange Pi 5에서는 "카메라 입력 -> VisionIPC -> modeld -> 메시지 발행" 체인이 살아 있다.

## 7. 남은 주요 이슈

- Panda 플래시/재연결 직후에는 일시적으로 `Lost panda connection while onroad`가 찍힐 수 있지만, 벤치용 `FORCE_IGNITION_ON`에서는 반복 로그를 억제하도록 보정했다.
- `timeout`이나 강제 종료 시 `modeld`/`onnx_runner` 파이프가 닫히는 케이스가 있어, 이를 assert 대신 정상 종료로 완화했다.
- `ping: socket: Operation not permitted`는 Armbian 권한/capability 문제로 보이며 현재 bring-up blocker는 아니다.
