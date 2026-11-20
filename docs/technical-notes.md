# JetBot Mini 스테레오 매핑 실험 기록

처음 실행할 때는 [README의 단계별 안내](../README.md)를 사용하세요. 아래 내용은 여러 시점의 실험 메모입니다. 기록에 언급된 원본 영상·실내 지도·로그는 실험 장치에 보관했습니다.

JetPack의 `nvarguscamerasrc` → ROS 2 좌/우 영상·CameraInfo → `image_proc`
영상 정렬 → RTAB-Map 스테레오 오도메트리·SLAM 구성입니다.

확인된 장치: Jetson Nano Developer Kit / L4T R32.7.1 (JetPack 4.6.1 계열),
CSI IMX219 카메라 2대. 2026-11-17 장착 변경 후 렌즈 중심 간격 실측 106~107 mm,
현재 보정 추정 간격 108.93 mm입니다. 이전 57 mm 장착 자료는 과거 기록입니다.
시야각은 사용자 추정 약 77°이며
수평/대각 기준과 실제 렌즈 사양은 미확인입니다. 거리 계산에는 보정된 내부 파라미터를 사용합니다.

현재 상태: Nano의 기존 Humble 컨테이너에서 카메라 영상을 발행하고,
노트북 Jazzy에서 정렬 영상 수신과 RTAB-Map 지도 DB 저장을 확인했습니다.
넓은 공간의 지도 정확도와 루프 폐쇄 검증은 남아 있습니다.
현재 작업 PC는 Ubuntu 24.04 / x86_64 / ROS 2 Jazzy이며 카메라가 연결되어 있지 않습니다.

## 현재 동작 확인된 ROS 2 카메라 연결

### 바퀴 수동 주행

노트북의 일반 터미널에서 `bash jetbot_drive.sh status`로 현재 배터리와 드라이버 상태를 확인합니다.
`bash jetbot_drive.sh teleop`는 기존 Nano 드라이버를 활성화하고 키보드 조작을 시작합니다.
W/S 전진·후진, A/D 회전, Space/X 정지, Q 종료입니다. 첫 방향 확인은 바퀴를 띄운 상태에서 합니다.
주행 명령이 끊기면 기존 드라이버의 0.3초 타임아웃으로 정지합니다.
별도 터미널의 `bash jetbot_drive.sh stop`은 정지 후 드라이버를 비활성화합니다.
실제 속도는 엔코더로 검증하지 않았으며 현재 PWM 설정에 따라 달라집니다.
저전압 보호 기준은 기존 설정 9.9 V를 유지합니다. 오류 보호를 우회하지 않습니다.
현재 SSH 공유 연결 기본 경로는 `/tmp/jetbot-small-area`이며
`JETBOT_SSH_CONTROL_PATH` 환경변수로 바꿀 수 있습니다.

### 모터 보드 전압 오보 (2026-11-20 해결)

**증상:** 드라이버를 시작하고 약 4분 19초 뒤 `MOTOR_WORKER_FAULT RuntimeError: Motor battery missing/low: 1.33 V`가 발생해 구동이 차단됐습니다. 사용자가 배터리 상태가 정상임을 확인했습니다.

**원인 (Nano에서 실측):** 모터 보드 STM8(I²C 버스 1, 주소 0x1B)은 마지막 I²C 쓰기 이후 **256번째 전압 읽기부터 4~20회 동안 상위 바이트를 0으로** 보냅니다.
전압 원시값은 `V = ((hi << 8) | lo) × 13.3 / 1023`으로 계산합니다(Yahboom 원본 공식).
상위 바이트가 빠지면 11.31 V(`0x366`)가 1.33 V(`0x066`)로 보고됩니다.

| 시험 | 결과 |
|---|---|
| 1,600회 연속 읽기 (약 20 Hz) | 0부터 센 255, 511, 767, 1023, 1279, 1535번째 읽기에서 상위 바이트 0. 간격 정확히 256 |
| 120회 읽기 → 정지 쓰기 → 계속 읽기 | 오류가 375번째(=120+255)에서 발생. 쓰기가 카운터를 초기화함 |
| 200회마다 부저 OFF(`0x06 ← 0`) 쓰기 | 900회 읽기 중 오류 0회 |
| 기존 `docker logs` | 정지 14건 중 9건이 시작 후 4분 18~20초 (1초 주기 × 256회). 로그의 저전압 값 0.61~2.00 V에 상위 바이트 `0x03`을 붙이면 모두 10.60~11.99 V |

칩 내부에 8비트 읽기 카운터가 있다는 설명은 측정 결과에서 추정한 모델입니다(STM8 펌웨어 비공개).

**ROS1 → ROS2 이식에서 드러난 이유:** Yahboom ROS1 원본은 30초마다 읽고 9.9 V 이하를 `Battery_Empty` 상태 표시로만 썼습니다. 그래서 2시간 8분에 한 번 표시가 잠깐 튀는 정도였습니다. ROS2 이식본은 1초마다 읽고, 한 번이라도 9.9 V 미만이면 모터 worker 프로세스를 종료했습니다.

**수정 (Nano `/home/ros2/jetbot_ros2`, 이미지 재빌드 완료):**

- `jetbot_mini_ros2/core.py` `I2CBus.voltage()`: 200회 읽을 때마다 부저 OFF를 써서 보드 카운터를 초기화합니다(`REFRESH_READS = 200`). 상위 바이트가 0인 값(3.33 V 미만)은 `BatteryReadInvalid`로 처리합니다.
- `BatteryMonitor`: 9.9 V 미만이 5회 연속(`battery_low_samples`, 기본 5)이면 정지합니다. 무효값만 10초(`battery_invalid_timeout`, 기본 10) 이어지면 `Motor battery missing or reading invalid`로 정지합니다.
- `read_voltage()`: `jetbotctl battery`와 `jetbotctl mode real`은 유효 샘플 5개의 중앙값을 사용합니다.
- 검증: 안전 테스트 15개 통과(호스트 Python 3.6, 이미지 Python 3.10). 새 드라이버 7분 29초 연속 실행 중 fault 없음(10.61 V). 실제 주행 시험은 남아 있습니다.
- 되돌리기: 원본은 `/home/ros2/jetbot_ros2.bak-20261120-battery`, 이전 이미지는 `jetbot-mini-ros2:before-battery-fix` 태그입니다.

그림 설명 자료는 [`jetbot-256th-read.pdf`](jetbot-256th-read.pdf)에 있습니다.

2026-11-17 수동 이동 지도는 `maps/jetbot-20261117-124440.db`에 정상 종료 후 저장했고,
`maps/jetbot-20261117-124440-saved-backup.db`로 백업했습니다. SQLite 무결성 검사 결과는
`log/mapping-saved-106mm.json`에 있습니다. 주행 준비 과정에서 매핑은 종료했고 카메라 연결은 유지합니다.

노트북의 지도 생성 패키지 설치는 `bash install_rtabmap.sh`로 진행합니다.
Ubuntu 24.04 / ROS 2 Jazzy를 확인한 뒤 `ros-jazzy-rtabmap-ros`와 `ros-jazzy-image-proc`를
설치합니다. sudo 인증은 노트북 터미널에서 입력하며, 완료 기록은
`log/rtabmap-install-verification.txt`에 저장됩니다.

설치 후 첫 지도 생성은 노트북에서 `bash start_mapping.sh`로 실행합니다.
SSH 인증이 필요하면 노트북 터미널에서 입력합니다. 기존 카메라 연결을 시작하고
좌우 정렬 영상과 CameraInfo가 들어오는지 확인한 뒤 RTAB-Map을 실행합니다.
이미 노트북 변환 노드가 정렬 영상을 발행하므로 image_proc 정렬기를 중복 실행하지 않습니다.
기본으로 RTAB-Map 창을 표시하며 `--no-viz`를 추가하면 GUI 없이 실행합니다.
수신·동기화 대기열은 각각 5개이며 `--queue-size=10`처럼 변경할 수 있습니다.
대기열 축소는 밀린 영상의 누적을 제한하지만 영상 처리 자체를 빠르게 하지는 않습니다.
106~107 mm 장착용으로 시차 탐색 범위는 256픽셀로 설정했습니다
(`max_disparity` launch 인자). 이전 128픽셀 설정에서 실패하던 현재 장면의 추적 회복을 확인했습니다.
이는 모든 거리·장면에서 추적을 보장하는 최소 거리 사양은 아닙니다.
지도는 프로젝트 `maps/jetbot-날짜-시간.db`에 저장합니다. `Ctrl+C`로 정상 종료하며
카메라 스트림까지 중지하려면 별도로 `bash ros_cameras.sh stop`을 실행합니다.
기존 지도를 다시 열려면 `bash start_mapping.sh maps/기존파일.db`로 지정합니다.

JetPack 호스트에서 GStreamer로 두 CSI 카메라를 열고, 같은 파이프라인의 프레임 시간과
JPEG를 로컬 TCP(127.0.0.1:8765)로 전송합니다. 기존 `jetbot-mini-ros2` Humble 컨테이너는
압축 영상과 CameraInfo를 발행합니다. 호스트 네트워크와 기존 보안 설정을 유지하며
카메라용 NVIDIA 라이브러리나 영상 패키지를 컨테이너에 추가하지 않습니다.
노트북 Jazzy에서 JPEG를 풀고 보정 행렬로 영상을 정렬합니다. 모터 드라이버의 활성화 상태는 변경하지 않습니다.

실제 장치에서 640×480, 좌우 각 15 fps, 8초간 원본/정렬 토픽 각각 약 118프레임을 확인했습니다.
검증 기록과 영상은 `captures/ros-live-check/`에 있습니다. 수신한 간격은 57.74 mm입니다.
ROS 도메인은 기존 컨테이너와 동일한 **16**을 사용합니다.

노트북의 프로젝트 폴더에서:

```bash
bash ros_cameras.sh start
bash ros_cameras.sh status
bash ros_cameras.sh stop
```

시작은 기존 설치 경로(`/home/ros2/jetbot_stereo_camera`)를 사용합니다. 필요한 경우 SSH 비밀번호를
입력하며 공유 연결을 재사용합니다. 컨테이너를 새로 만들었다면 호스트의 카메라 코드를 다시 복사합니다.
이 스크립트는 부팅 자동 실행을 등록하지 않습니다.
노트북에 사용자 systemd가 있으면 영상 변환 노드를 `jetbot-stereo-decoder.service`라는
일시적 사용자 서비스로 실행해 시작 터미널이 끝나도 유지합니다. 상태는
`systemctl --user status jetbot-stereo-decoder.service`로 볼 수 있습니다.
원격 카메라 시작은 수신 포트가 실제로 열렸는지 확인한 뒤 진행하며, 촬영 프로그램은
연결 실패 시 최대 30초간 재시도합니다. 중지 후 재시작할 때도 이전 발행 노드가 완전히
종료된 것을 확인합니다.

JetBot 컨테이너의 토픽:

* `/stereo/left/image_raw/compressed`, `/stereo/right/image_raw/compressed`
* `/stereo/left/camera_info`, `/stereo/right/camera_info`

노트북 변환 노드가 추가하는 토픽:

* `/stereo/left/image_raw`, `/stereo/right/image_raw`
* `/stereo/left/image_rect`, `/stereo/right/image_rect`

노트북에서 영상 보기:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=16
ros2 run rqt_image_view rqt_image_view
```

로그는 JetBot 호스트의 `/home/ros2/jetbot_stereo_camera/capture.log`,
컨테이너의 `/tmp/jetbot-stereo-publisher.log`, 노트북의 `/tmp/jetbot-stereo-decoder.log`에 있습니다.
카메라가 이 구성에서 실행 중이면 사진/동영상 촬영 스크립트가 같은 CSI 센서를 동시에 열지 않도록
먼저 `bash ros_cameras.sh stop`으로 중지합니다.
프레임 시간은 버퍼 PTS를 공통 시계에서 변환한 값이며 하드웨어 노출 동기화를 뜻하지 않습니다.

## 현재 촬영 방식: 동영상 한 번 → 프레임 자동 선별

노트북에서 다음 명령 하나를 실행합니다. JetBot에는 ROS/Docker 없이 JetPack GStreamer를 사용합니다.

```bash
bash record_stereo_video.sh 30
```

두 CSI 카메라를 하나의 GStreamer 파이프라인에서 30초 녹화하고 프레임별 시간 기록을 함께
가져옵니다. `captures/videos/촬영번호/`에 `camera-0.avi`, `camera-1.avi`, 타임스탬프 CSV,
`recording.json`을 저장하고 PC에서 자동으로 프레임을 선별합니다.
각 영상은 640×480, 15 fps MJPEG AVI입니다. 체크무늬가 양쪽에 보이게 유지하며 자세를 천천히
바꾸고 중간에 2~3초 정지하면 됩니다. 카메라 사이 장착 관계와 패턴 크기는 바꾸지 않습니다.

프레임 시간차 35 ms 이내, 패턴 인식 성공, 움직임 속도와 실제 시간차로 추정한 오차 0.5 px 이하,
선명도와 자세 차이를 모든 프레임에서 검사합니다. 보정 후보는 `frame-pairs/`에만 저장하며, 탈락한 프레임은
별도 사진으로 저장하지 않습니다. 검사 내역은 `selection-report.json`에 남깁니다.
이 수치는 프레임 선별을 위한 프로젝트 기준이며, 센서의 하드웨어 노출 동기화를 의미하지 않습니다.
파일의 25 mm 기본값과 실제 표시한 패턴 크기가 같아야 합니다. 기존 17.286 mm 자료와 섞지 않습니다.

`calibration/left.yaml`, `right.yaml`이 있으면 선별된 마지막 프레임의 패턴 중심 거리도 자동 계산합니다.
거리 결과는 해당 프레임 폴더의 `distance.json`에 저장합니다.

2026-11-16 촬영한 기존 25 mm 사진 12쌍의 첫 보정 결과는 카메라 간격 59.62 mm,
stereo RMS 1.710 px, 정렬 후 수직 오차 평균 2.726 px로 품질 검사를 통과하지 못했습니다.
파일 수가 부족하다는 이유로 반복 촬영하는 대신, 촬영 안정성과 좌우 시간 차이를 진단합니다.
기존 자료는 보존하며, 이 실패 결과를 유효한 거리 보정값으로 사용하지 않습니다.

이후 기존 12쌍으로 렌즈 내부 파라미터를 구하고, 사용자 녹화 영상
`captures/videos/20261116-152551-bI1JKs`에서 시간차가 작은 프레임 5쌍을 골라
좌우 관계를 보정했습니다. 간격 57.74 mm, stereo RMS 0.242 px, 평균 수직 오차 0.216 px로
프로젝트 품질 기준을 통과했고 보정 파일을 `calibration/`에 적용했습니다.
보정에 쓰지 않은 인접 프레임 1쌍은 수직 오차 0.283 px, 패턴 중심 거리 약 40.14 cm였습니다.
이것은 같은 영상 내 점검이며 독립적인 실측 거리 검증은 아닙니다. 절대 거리 정확도는 자로 확인해야 합니다.
상세 보고서는 해당 영상의 `frame-pairs-reviewed/calibration-result/`에 보존합니다.

## 1. JetBot 본체 정보 확인

### 노트북에서 한 번에 사진 촬영·가져오기

노트북 터미널에서 프로젝트 폴더의 `bash take_photos.sh`를 실행합니다.
기본 SSH 대상은 현재 PC에 등록된 `ros2`이며, 비밀번호 인증도 지원합니다.
JetBot에 스크립트를 따로 복사하거나 ROS/Docker를 실행할 필요가 없습니다.

```bash
bash take_photos.sh
# 다른 주소 또는 저장 위치:
bash take_photos.sh ros2@192.0.2.10 ./captures
```

한 번의 SSH 연결로 센서 0과 1을 동시에 열고 각각 30프레임 중 마지막 JPEG를 가져옵니다.
노트북의 `captures/날짜-시간-고유번호/`에 `camera-0.jpg`, `camera-1.jpg`와 로그를 저장합니다.
카메라를 사용하는 다른 프로그램은 먼저 종료하고, 촬영 중에는 로봇과 대상을 멈춰주세요.
이 동시 실행은 하드웨어 동기화를 보장하지 않습니다. 각 실행의 사진을 별도 폴더에 보존합니다.

### 현재 화면 패턴으로 안내에 따라 스테레오 보정

보조 모니터용 `assets/external-checkerboard-25mm.html`을 보조 화면으로 옮긴 뒤
전체화면 버튼으로 표시합니다. 실제 출력은 HDMI-1-0 / 1920×1080 / 60 Hz,
모니터가 보고한 표시 영역은 598×336 mm입니다. 목표 한 칸 크기는 25 mm이며,
전체 패턴은 225×175 mm입니다. 전체화면에서도 주변에 넓은 흰 여백을 남깁니다.
가로·세로 100 mm 눈금을 자로 측정하고 필요하면 화면의 보정 기능을 사용하세요.
그 뒤 화면 배율과 패턴 크기를 유지하고 노트북에서 실행하세요.

```bash
bash calibrate_stereo.sh
```

기본 칸 크기는 25 mm입니다. 세션마다 `capture-config.json`에 크기를 저장하며
보정 계산 시 읽습니다. 이전 7칸=121 mm 패턴은 한 칸 17.285714 mm이므로 새 25 mm
패턴의 사진과 섞지 않습니다. 이전 크기로 새 세션을 촬영하려면
`SQUARE_MM=17.285714 bash calibrate_stereo.sh`를 사용하세요.

SSH 비밀번호를 한 번 입력하면 20가지 자세가 순서대로 표시됩니다.
매번 화면/로봇 전체의 자세를 바꾸고 멈춘 뒤 Enter를 누릅니다. 카메라 사이 장착 관계는
바꾸지 않습니다. 양쪽 패턴 인식 실패 또는 거의 동일한 자세는 다시 촬영하도록 안내합니다.
제외된 사진 쌍은 양쪽 원본 및 교차점 표시 사진을 자동 삭제합니다. 정상 사진과 촬영 로그는
유지하며, 각 제외 폴더의 `rejection.json`에 이유와 삭제 파일명을 기록합니다.
사진을 보존하며 수동 검사하려면 Python 검사 명령에 `--keep-rejected`를 추가하세요.
각 자세에서 화면을 평면으로 유지하고 화면 확대나 패턴 크기를 바꾸지 않습니다.
최소 12쌍의 서로 다른 유효 사진이 필요하며, 여러 거리와 기울기에서 20쌍을 목표로 합니다.

사진은 `captures/calibration-sessions/촬영번호/`에 저장합니다. 계산 후 같은 폴더의
`calibration-result/report.json`에 오차, 추정 간격, 사용한 촬영 목록을 기록합니다.
재투영 RMS 1.5 px, 평균 수직 오차 1 px / 95백분위 2 px, 실측 간격 대비 25%,
기울기 변화 최소 10도를 초기 품질 기준으로 사용합니다. 이 값들은 본 프로젝트의
검사 기준이며, 정확도를 보장하는 공인 기준은 아닙니다. 오차는 보정에 쓴 영상에서 계산합니다.
기준을 통과하면 `left.yaml`, `right.yaml`과 정렬 미리보기를 생성하며 기존 보정 파일을
덮어쓰지 않습니다. 결과를 검토한 뒤 `calibration/`에 복사해야 매핑에 적용됩니다.
안내 스크립트는 보정 후 마지막 사진 쌍에서 왼쪽 렌즈 중심부터 패턴 중심까지의
직선거리와 전방 깊이를 계산합니다. 결과는 사진 폴더의 `distance.json`에 저장합니다.
이 값은 보정 영상에서 얻은 추정치이므로 실제 자로 잰 거리와 독립적으로 비교해야 합니다.

보정 후 다른 사진 쌍의 패턴 거리를 측정하려면:

```bash
/usr/bin/python3 scripts/measure_stereo.py captures/사진번호 \
  --calibration-dir captures/calibration-sessions/촬영번호/calibration-result --square-mm 25
```

기존 세션을 다시 검사하거나 계산하려면:

```bash
/usr/bin/python3 scripts/calibrate_stereo.py captures/calibration-sessions/촬영번호 --check-only
/usr/bin/python3 scripts/calibrate_stereo.py captures/calibration-sessions/촬영번호
```

양쪽 사진은 소프트웨어로 동시에 시작할 뿐 하드웨어 동기화가 아닙니다. 마지막 프레임의
노출 시각이 다를 수 있으므로 촬영 시작 전부터 완료 때까지 장면을 정지시켜야 합니다.
보정 계산은 PC의 OpenCV로 수행하므로 JetBot에 ROS 또는 Docker 설치가 없어도 가능합니다.
추정 방식은 [OpenCV 보정 API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)를 사용합니다.

먼저 이 프로젝트를 JetBot에 복사한 뒤 호스트에서 다음 검사를 실행할 수 있습니다.
두 카메라를 사용하는 다른 프로그램을 종료한 상태에서 실행하세요.

```bash
bash scripts/check_csi.sh
```

이 검사는 GUI 없이 두 CSI 센서를 동시에 열어 각각 30프레임을 받고 마지막 JPEG와
로그를 `/tmp/jetbot-camera-check-날짜-시간/`에 저장합니다. PASS는 영상 파일 수신 성공을
의미하며, 저장 영상을 열어 초점·방향·겹치는 시야를 별도로 확인해야 합니다.
필요하면 `WIDTH=1280 HEIGHT=720 FPS=30 bash scripts/check_csi.sh`처럼 지원 모드로 바꿉니다.
센서 ID는 `LEFT_SENSOR_ID`와 `RIGHT_SENSOR_ID` 환경 변수로 지정할 수 있습니다.

JetBot에서 실행하세요.

```bash
cat /etc/nv_tegra_release
tr -d '\0' </proc/device-tree/model
ls /dev/video*
gst-inspect-1.0 nvarguscamerasrc
docker info
```

CSI 커넥터가 있어도 카메라의 센서 드라이버와 Argus 지원이 필요합니다.
카메라 2개를 동시에 열 수 있는지도 확인해야 합니다. 멀티플렉서로 번갈아 촬영하는
보드는 이 구성의 동시 스테레오 입력으로 바로 사용할 수 없습니다.

아래 명령을 각각 별도 터미널에서 실행해 동시 촬영 여부를 확인합니다.
지원 모드·해상도·프레임 속도는 카메라 모델에 맞게 바꾸세요.

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=100 ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1' ! fakesink
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=100 ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1' ! fakesink
```

## 2. Docker 이미지 결정 및 빌드

`.env.example`을 `.env`로 복사하고 `JETSON_ROS_BASE_IMAGE`에 실제 이미지 주소를 입력합니다.
호스트 JetPack/L4T 및 ARM64와 호환되고, ROS 2·ROS apt 저장소가 설치되어 있으며,
NVIDIA 카메라 GStreamer 플러그인을 사용할 수 있는 이미지여야 합니다.
일반 `ros:humble`/`ros:jazzy` 이미지만으로 Argus 사용을 보장할 수 없습니다.
Dockerfile은 기존 ROS 환경에 필요한 패키지를 추가합니다.

이 구성은 ROS 2 Humble/Jazzy 계열을 대상으로 작성했습니다. JetPack 4 기반 Nano라면
해당 ROS 버전과 NVIDIA 사용자 공간 라이브러리를 함께 지원하는 기반 이미지나
별도의 빌드 방법부터 확인해야 합니다. 현재 특정 이미지 태그의 호환성을 검증하지 않았습니다.

```bash
cp .env.example .env
# .env의 JETSON_ROS_BASE_IMAGE 값을 편집한 다음:
docker compose build
docker compose run --rm stereo gst-inspect-1.0 nvarguscamerasrc
docker compose run --rm stereo python3 -c 'import cv2; print(cv2.getBuildInformation())'
```

OpenCV 출력에서 `GStreamer: YES`를 확인합니다. `pip opencv-python`으로 시스템 OpenCV를
덮어쓰지 마세요. NVIDIA 컨테이너 런타임이 필요하며, 호스트 Argus 소켓을 컨테이너에
연결합니다. Compose는 로컬 로봇의 카메라 접근을 위해 privileged 모드로 실행합니다.
원격 접속용 포트를 공개하지 않습니다.

## 3. 좌/우 영상 확인

왼쪽/오른쪽은 로봇이 바라보는 방향을 기준으로 정합니다.

```bash
docker compose run --rm --name jetbot-capture stereo \
  ros2 launch jetbot_stereo stereo.launch.py \
  left_sensor_id:=0 right_sensor_id:=1 width:=640 height:=480 fps:=15
```

다른 터미널에서:

```bash
docker exec jetbot-capture /bin/bash /entrypoint.sh ros2 topic list
docker exec jetbot-capture /bin/bash /entrypoint.sh ros2 topic hz /stereo/left/image_raw
docker exec jetbot-capture /bin/bash /entrypoint.sh ros2 topic hz /stereo/right/image_raw
```

촬영 노드는 좌우 각각 별도 스레드로 동작합니다. `sensor_mode`와 `flip_method`도
launch 인수로 바꿀 수 있습니다. 두 영상이 같은 방향이고 시야가 충분히 겹쳐야 합니다.
초점, 카메라 간 거리, 방향을 고정하고 이후 보정 중이나 매핑 중에 바꾸지 마세요.

## 4. 스테레오 공동 보정

두 카메라를 **함께** 보정해야 실제 거리 척도와 정렬 정보를 얻을 수 있습니다.
각 카메라의 단독 보정 파일 두 개만으로는 충분하지 않습니다.
예시는 내부 교차점 8×6, 칸 크기 0.025 m의 체커보드입니다. 실제 보드에 맞게 수정하세요.

캡처를 실행한 상태에서 Jetson의 GUI 세션에서 다음 명령을 실행합니다.
GUI 프로그램을 컨테이너에서 표시할 X11 인증 설정이 필요합니다.
데스크톱과 X 서버 접근 권한이 없는 환경에서는 먼저 GUI 사용 방법을 설정해야 합니다.

```bash
docker exec -it jetbot-capture /bin/bash /entrypoint.sh \
  ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --approximate 0.02 --no-service-check \
  --ros-args -r left:=/stereo/left/image_raw -r right:=/stereo/right/image_raw \
  -r left_camera:=/stereo/left -r right_camera:=/stereo/right
```

양쪽 화면에 체커보드를 함께 보여주며 거리·위치·기울기를 다양하게 촬영하고
CALIBRATE → SAVE를 누릅니다. 이 드라이버에는 `set_camera_info` 서비스가 없으므로
COMMIT 대신 저장된 파일을 사용합니다.

```bash
docker cp jetbot-capture:/tmp/calibrationdata.tar.gz /tmp/jetbot-calibrationdata.tar.gz
tar -tzf /tmp/jetbot-calibrationdata.tar.gz
```

압축 안의 `left.yaml`, `right.yaml`을 프로젝트의 `calibration/`에 저장합니다.
보정 도구 버전마다 압축 내부 경로가 다를 수 있으므로 위 목록을 확인한 뒤 추출하세요.
오른쪽 `projection_matrix`의 수평 이동 항이 음수이고,
`-P[3]/P[0]`이 실측한 카메라 사이 거리(m)와 비슷한지 확인합니다.
정렬 후 같은 물체가 두 화면에서 같은 높이에 나타나는지도 확인합니다.
보정 때와 매핑 때 해상도·영상 뒤집기·센서 모드를 동일하게 유지합니다.

## 5. 매핑

촬영 전용 컨테이너를 Ctrl+C로 종료한 후 실행합니다. 두 프로세스가 같은 CSI 센서를
동시에 열지 않도록 합니다.

```bash
docker compose run --rm --name jetbot-mapping stereo \
  ros2 launch jetbot_stereo stereo.launch.py mapping:=true \
  calibration_dir:=/data/calibration width:=640 height:=480 fps:=15
```

기본은 GUI 없는 실행입니다. X11 인증을 설정한 환경에서는 `viz:=true`로 RTAB-Map 창을
사용할 수 있습니다. 지도는 호스트의 `maps/jetbot.db`에 유지되며 재실행해도 삭제하지 않습니다.
새 지도를 만들려면 `database_path:=/data/maps/다른이름.db`를 사용하세요.

주요 토픽은 `/stereo/left/image_rect`, `/stereo/right/image_rect`, `/stereo/odom`, `/map`입니다.
로봇을 천천히 움직여 특징이 충분한 벽·가구를 촬영하고, 출발 지점을 다시 보며 루프 폐쇄를 확인합니다.

```bash
docker exec jetbot-mapping /bin/bash /entrypoint.sh ros2 topic hz /stereo/odom
docker exec jetbot-mapping /bin/bash /entrypoint.sh ros2 topic info /map --verbose
docker exec jetbot-mapping /bin/bash /entrypoint.sh \
  ros2 run nav2_map_server map_saver_cli -f /data/maps/jetbot_2d --ros-args -r map:=/map
```

마지막 명령은 `/map`이 실제 발행된 뒤 2D 점유 지도를 저장합니다.
3D 지도와 촬영 기록은 RTAB-Map DB로 보존합니다. 종료 시 DB 쓰기가 끝나도록 정상 종료하세요.

## 시간 동기화 및 로봇 통합 범위

현재 캡처 노드는 프레임을 읽은 시각을 ROS 타임스탬프로 사용합니다. 이 값은 센서 노출 시각이
아니며 하드웨어 동기화도 아닙니다. RTAB-Map은 기본 20 ms 범위의 근사 동기화를 사용합니다.
움직이는 물체, 빠른 회전, 좌우 지연 차이는 거리 오차와 추적 실패를 유발할 수 있습니다.
허용 시간을 넓히는 것만으로 해결할 수 없습니다. 정밀도를 높이려면 카메라의 동시 노출 지원과
Argus 캡처 메타데이터를 이용한 타임스탬프 처리를 확인해야 합니다.

TF는 `map → stereo_odom → stereo_camera_link → stereo_left_optical_frame`입니다.
카메라 중심을 기준으로 매핑하며 로봇 본체까지의 장착 위치를 임의로 넣지 않았습니다.
오른쪽 영상의 기하 정보는 보정 P/R 행렬을 통해 전달합니다. 오른쪽 optical TF는
이 구성에서 발행하지 않습니다. 로봇 기준 지도·Nav2·바퀴 오도메트리 통합에는 실측한
`base_link → stereo_camera_link` 변환 및 기존 TF 발행 주체 확인이 추가로 필요합니다.
이 프로젝트에는 모터 구동이나 자율주행 노드가 포함되지 않습니다.

다른 CSI 드라이버가 ROS 영상을 이미 발행한다면 `start_capture:=false`로 캡처를 끄고
동일한 토픽·프레임·보정 정보를 공급할 수 있습니다.

## 개발 확인

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select jetbot_stereo
PYTHONPATH=src/jetbot_stereo /usr/bin/python3 -m unittest discover -s tests
source install/setup.bash
ros2 launch jetbot_stereo stereo.launch.py --show-args
```

참고: [NVIDIA CSI/Argus 문서](https://docs.nvidia.com/jetson/archives/r38.2/DeveloperGuide/SD/CameraDevelopment/CameraSoftwareDevelopmentSolution.html),
[ROS image_proc](https://github.com/ros-perception/image_pipeline/blob/rolling/image_proc/doc/tutorials.rst),
[RTAB-Map ROS 2 launch](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py).
NVIDIA 링크는 API 개요이며 보드에 설치된 L4T 버전의 문서와 함께 확인해야 합니다.

## 회전 촬영과 벽 평면 후보 분석

`scripts/rotate_stereo_survey.py`는 카메라·스테레오 오도메트리 실행 상태에서 실제 모터를 제어하고,
Nano가 송출하는 원본 JPEG와 프레임 시각·회전각을 저장합니다. 바퀴 엔코더 회전각이 아니라
영상 오도메트리의 누적 yaw를 사용합니다. 짧은 회전과 정지를 반복하며 추적이 끊기면
먼저 정지합니다. 정지 상태에서 유효한 pose 4개가 들어오면 재개하고, 3초 내 회복하지
못하거나 20cm 중심 이탈·카메라/상태 수신 중단·배터리 오류·다른 cmd_vel 발행자를 감지하면 종료합니다.
종료 시 0속도와 /jetbot/stop으로 모터를 비활성화합니다. 실제 회전은 테스트된 명령에서도
영상 추적 품질에 좌우되며 정확한 360도 완료를 보장하지 않습니다.

`scripts/detect_wall_planes.py <녹화폴더> --calibration-dir calibration`은 저장된 JPEG에
보정, StereoSGBM 시차, 좌우 일치·무늬·거리 검사를 적용하고 RANSAC 수직 평면 후보를 찾습니다.
평면 후보는 가구·칸막이·모니터일 수도 있으며, 벽이라는 의미 분류를 검증한 것은 아닙니다.
카메라가 대략 수평으로 장착됐다는 가정을 사용합니다. 유효 깊이가 화면의 3% 미만이면
평면 후보를 출력하지 않습니다. 이 값과 평면 크기·잔차 한계는 초기 실험용 기준입니다.
깊이 없는 픽셀은 검은색 미확인 영역이며 자유 공간으로 채우지 않습니다.
회전 녹화에 pose 기록이 있으면 각도 변화가 작은 정지 구간만 분석합니다.
출력은 녹화폴더의 `wall-analysis/report.json`, `wall-candidates.avi`, 프레임별 비교 JPG입니다.

2026-11-17 실험은 두 구간의 영상 추정 회전량 합 약 150도에서 추적 회복 실패로 중단했습니다.
360도 실험은 미완료이며 모터·카메라·매핑을 정지했습니다. 결과는
`log/rotation-wall-survey-summary.json`과 `captures/rotation-survey-20261117-145927/`,
`captures/rotation-survey-20261117-150057/`에 있습니다. `left.avi`, `right.avi`는 원본 영상이며
JPEG 원문과 프레임별 정확한 시각은 각각 MJPEG 파일과 events.jsonl에 별도로 보존합니다.

## 흰 가림판의 위·아래 경계로 벽 지도 생성

사진과 같은 밝은 무채색 가림판과 더 어두운 평평한 바닥을 대상으로 한 실험 모드입니다.
OpenCV의 median/bilateral 필터 → CLAHE 명암 보정 → Canny/LSD 선분 검출 →
위·아래 선분 짝짓기와 내부 명암·색상 검사를 사용합니다. 바닥과 맞닿은 아래 경계,
실측 카메라 높이, 카메라 내부 보정 및 장착 기울기로 수직 벽 가설을 만듭니다.
벽 경계가 보이지 않거나 지평선에 가까워 거리를 계산할 수 없으면 그 부분은 미확인으로 둡니다.
의자·책상 모서리 등도 선분 후보가 될 수 있어 일반 환경의 벽 의미 분류를 보장하지 않습니다.

현재 사용자 실측 카메라 높이는 9.8~10.0cm이며 기본값은 9.9cm입니다. 장착 기울기는
미확인이고 기본값은 하향 0도입니다. 바닥의 평탄함, 카메라 기울기·롤, 바닥 접촉점 판정이
틀리면 거리가 틀어집니다. 자로 잰 거리와의 검증은 아직 필요합니다. HC-SR04는 연결하지 않았습니다.

노트북의 프로젝트 폴더에서 실행합니다. 실제 하향 기울기를 알고 있다면 두 번째 값을 바꿉니다.

```bash
bash start_boundary_mapping.sh 9.9 0
```

카메라와 새 경계 지도 노드, 임시 영상 오도메트리, RViz 벽 표시를 시작합니다. 모터 명령은 보내지 않습니다.
기존 매핑 프로세스가 있으면 종료를 안내하고 시작하지 않습니다. Ctrl+C로 종료하면
`maps/boundary-날짜-시간.json`에 현재 벽 셀과 높이를 가진 벽 구간을 저장합니다.
일반 RTAB-Map DB를 새로 만들지 않으며 이 JSON은 Database Viewer용 DB 형식이 아닙니다.

- `/partition/wall_markers`: RViz용 3D 벽 가설
- `/partition/walls`: 경계·거리 가설·위치 추적 상태
- `/partition/left/image_rect`, `/partition/right/image_rect`: 배경을 가린 지도 입력
- `/partition/preprocessed`: 배경을 가린 경계선 디버그 화면
- `/partition/odom`: 영상 오도메트리

배경을 추적 영상에서도 가리면 이 저장 사진의 유효 특징점이 부족해 초기화에 실패했습니다.
따라서 전처리한 전체 영상은 위치 추적의 임시 메모리에서만 사용하며,
추적용 local feature map은 `OdomF2M/MaxSize=1000`으로 제한합니다.
지도 파일에는 배경 이미지·점군·특징점을 저장하지 않습니다. 벽 위의 배경은 지도 입력에서 지우며,
벽 셀만 저장하고 벽 뒤나 미관측 영역의 셀은 만들지 않습니다. 디버그 이미지도 배경을 가려 저장합니다.
벽이 없는 부분을 자유 공간으로 채우지 않습니다. 최대 벽 셀 50,000개, 벽 구간 256개로 제한합니다.

사용자가 보여준 저장 노드 149에서 두 벽과 33개 벽 셀을 얻었습니다.
`maps/boundary-preview-20261117/processing-preview.jpg`와 `wall-map-preview.png`를 확인하세요.
원본 DB는 수정하지 않았습니다. 격리 ROS 도메인 202에서 정지 사진을 반복 재생해
실제 스테레오 오도메트리와 벽 저장 경로를 확인한 결과는
`log/boundary-real-odom-validation.json`에 있습니다. 실제 이동 중 경계 추적은 아직 검증하지 않았습니다.

재생 검사는 Nano/모터를 사용하지 않습니다. 프로젝트 폴더에서:

```bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
ROS_DOMAIN_ID=202 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/replay_boundary_ros.py
```

실행 지연 보완: 벽 검출 결과를 최대 20프레임 보관하고, 오도메트리의 촬영 시각과
25ms 이내로 맞는 프레임에만 좌표 변환을 적용합니다. 처리 지연 때문에 최신 영상과
이전 위치를 비교해 벽 저장이 누락되던 문제를 수정했습니다. 실제 실행 검증은
`log/boundary-live-fix-validation.json`에 있으며, 이때 벽 셀과 마커 송출 및 RViz 구독을 확인했습니다.

### 같은 벽의 반복 관측과 회전 방향

회전 방향은 `partition_odometry`가 이전/현재 전처리 영상의 특징점을 비교한 결과에서 얻습니다.
바퀴 엔코더나 모터 회전각 센서를 읽는 방식은 아닙니다. `/partition/walls`의 `visual_heading`에
출발 방향 기준 누적 yaw, 처리 프레임 간 회전각, left/right/stationary가 표시됩니다.
영상 추적이 불확실하면 `heading_valid`와 `pose_valid_for_map`을 확인해야 합니다.

벽 관리는 각도/거리 구간을 키로 누적하던 방식을 고정 ID의 벽 추적으로 바꿨습니다.
같은 세계 좌표계에서 방향 차이 7도, 수직 거리 차이 6cm, 선분 간격 10cm 이내인 관측은
기존 벽에 연관시켜 위치·범위·높이를 갱신합니다. 실제로 다른 벽이 가까이 있거나 영상 위치
추정 오차가 크면 이 기하 기준만으로 완전한 구분을 보장할 수 없습니다.
서로 떨어진 평행 벽·모서리는 분리하며 새 후보는 서로 다른 3프레임에서 관측된 뒤 표시합니다.
위치가 갱신된 벽의 셀은 현재 벽에서 다시 계산하므로 예전 위치의 셀을 계속 쌓지 않습니다.

RViz는 동일 `wall_hypotheses` namespace와 고정 ID로 Marker.ADD를 보내 기존 벽을 갱신합니다.
전체 DELETEALL은 시작 때 한 번만 보내고, 합쳐져 사라진 ID에만 DELETE를 보냅니다.
추적 결과가 갑자기 초기화/도약한 것으로 보이면 기존 벽을 유지하고 새 벽 갱신을 보류합니다.
`/partition/walls`에 `wall_count`, `wall_ids`, `map_hold_reason`을 추가했습니다.
현재 방식은 RTAB-Map 전체 SLAM의 재방문 최적화를 구현한 것은 아니며 긴 경로의 누적 오차는 남습니다.

회전 좌표 변환 후 같은 벽의 ID 유지, 반복 잡음, 벽 연장/분리, 기존 셀 잔상 제거,
회전각 ±180도 전환과 추적 좌표 도약 검사를 추가했습니다. 실제 이동 없이 정지 저장 영상을
ROS로 반복 재생해 벽 2개가 ID 0/1로 유지되는지도 검사합니다.

RViz 표시 시점은 `partition_map_view`를 따라가도록 설정했습니다. 지도 표시 보조 노드가 저장된
벽 구간들의 경계 상자 중심을 계산하므로 로봇이 원점에서 멀어져도 지도가 시야 밖으로 사라지지 않습니다.
`boundary_map_origin -> partition_odom`은 임의의 지도 원점을 등록하는 항등 변환이며 로봇 위치를
대신 만들어 주는 변환은 아닙니다. 현재 카메라 추적이 중단돼도 저장된 벽을 볼 수 있습니다.
`log/rviz-visible-20261118.png`는 실제 RViz 창에서 벽 표시와 Global Status Ok를 확인한 화면입니다.

## 기존 방 지도에서 위치 추정

노트북에서 `bash start_reference_localization.sh`를 실행합니다. RViz의 **2D Pose Estimate**로
지도에서 실제 출발 위치를 누른 채 카메라가 향하는 방향으로 드래그합니다. 카메라의 대략적인
위치와 방향을 초기값으로 줘야 합니다. 위치 추정 화살표는 `/amcl_pose`에서 확인할 수 있습니다.

이 모드는 `maps/map2-room-reference-20261118/map2_room.yaml`을 고정 지도로 사용합니다.
시각 오도메트리는 정렬 영상에서 상대 이동을 계산합니다. AMCL은 카메라가 검출한 가림판의
바닥 경계를 거리 관측으로 변환해 지도와 비교합니다. 경계를 확인하지 못한 방향은 미관측으로
처리합니다. 현재 벽 거리 설정은 카메라 높이 9.9cm, 하향 기울기 0도입니다.
평행한 벽 하나만 관측하면 벽을 따라 어느 위치에 있는지 구분하기 어렵습니다.
