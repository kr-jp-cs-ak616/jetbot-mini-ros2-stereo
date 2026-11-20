# JetBot Mini 스테레오 카메라와 ROS 2 매핑

Jetson Nano에 달린 IMX219 카메라 두 대로 사진을 찍고, 두 영상의 차이로 거리를 계산한 뒤, 노트북에서 ROS 2와 RTAB-Map으로 지도를 만드는 프로젝트입니다. 먼저 사진 두 장을 가져오는 것부터 시작하세요. 사진 촬영, 카메라 보정, 손으로 움직이는 매핑에는 모터가 필요 없습니다.

| 준비물 | 이 프로젝트에서 사용한 구성 |
|---|---|
| 로봇 | Yahboom JetBot Mini, Jetson Nano Developer Kit, JetPack 4.6.1 계열(L4T R32.7.1) |
| 카메라 | CSI IMX219 두 대, 좌우 공통 해상도 640×480, 초당 15프레임 |
| 노트북 | Ubuntu 24.04, ROS 2 Jazzy, OpenCV, RTAB-Map, RViz |
| 연결 | Nano와 노트북을 같은 로컬 네트워크에 연결하고 노트북에서 SSH 접속 |
| 보정 도구 | 체커보드를 띄울 화면과 자 |

Nano의 JetPack이 두 카메라에서 영상을 읽습니다. Nano의 ROS 2 Humble 컨테이너가 영상을 보내면, 노트북의 Jazzy가 영상 왜곡을 보정하고 지도와 위치를 계산합니다. 사진이나 보정 영상만 녹화할 때는 Nano의 Docker 컨테이너도 필요 없습니다.

## 1. 폴더 열기와 Nano 접속

1. GitHub에서 **Code → Download ZIP**을 눌러 파일을 받은 뒤 압축을 풉니다. Git을 쓰는 경우 **Code → HTTPS**에 표시된 주소를 복사해 `git clone 주소`를 실행할 수도 있습니다.
2. Ubuntu 파일 관리자에서 압축을 푼 폴더를 엽니다. 같은 폴더에 `take_photos.sh`가 보여야 합니다. 빈 곳을 우클릭해 **터미널에서 열기**를 선택합니다. 모든 **노트북** 명령은 이 폴더에서 실행합니다.
3. Nano의 로컬 터미널에서 `hostname -I`로 IP 주소를 확인하고 `whoami`로 로그인 이름을 확인합니다.
4. 노트북에서 `mkdir -p ~/.ssh`와 `nano ~/.ssh/config`를 차례로 실행합니다. 아래 세 줄을 추가하세요. `192.0.2.10`은 예시 주소입니다. `User`에는 Nano의 실제 로그인 이름을 넣습니다. 같은 이름의 `Host ros2`가 이미 있으면 해당 항목을 수정하세요. 편집기에서 `Ctrl+O`, `Enter`, `Ctrl+X`로 저장하고 나옵니다.

```sshconfig
Host ros2
    HostName 192.0.2.10
    User ros2
```

노트북에서 `ssh ros2`를 실행하세요. 비밀번호를 타이핑할 때 글자가 표시되지 않는 것은 정상입니다. 접속했다면 아래를 실행합니다.

```bash
cat /etc/nv_tegra_release
ls /dev/video*
gst-inspect-1.0 nvarguscamerasrc
exit
```

`R32`와 `REVISION: 7.1`이면 실험한 Nano와 같은 L4T 버전입니다. `exit`를 입력하면 노트북 터미널로 돌아옵니다.

## 2. 두 카메라에서 사진 가져오기

**노트북** 프로젝트 폴더에서 실행합니다.

```bash
bash take_photos.sh
```

출력에 표시된 `captures/날짜-시간-번호/` 폴더에 `camera-0.jpg`와 `camera-1.jpg`가 생깁니다. 두 사진을 열어 같은 물체가 보이는지, 초점이 맞고 위아래 방향이 같은지 확인하세요. 로봇의 진행 방향을 바라볼 때 왼쪽에 달린 카메라가 센서 0이 되도록 장착과 연결을 확인합니다. 한쪽 렌즈 앞을 잠깐 가린 채 찍으면 두 사진의 센서 순서를 구분할 수 있습니다.

이미 실시간 카메라가 실행 중이면 먼저 노트북에서 `bash ros_cameras.sh stop`을 실행합니다. 사진·동영상 촬영과 실시간 카메라는 같은 CSI 센서를 동시에 열 수 없습니다.

## 3. 체커보드 영상으로 거리 보정

보정은 좌우 영상의 렌즈 왜곡과 두 렌즈 사이의 위치 관계를 계산하는 과정입니다. 렌즈 중심 간격을 자로 재서 적어두세요. 이 프로젝트의 현재 장착은 **106~107 mm**였습니다. 이전 실험은 57 mm였습니다. 장착이나 해상도가 다르면 새 보정값을 사용하세요.

**노트북**에 영상 계산 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-yaml
```

1. 보조 모니터에서는 [25 mm 체커보드](assets/external-checkerboard-25mm.html)를 열고 전체화면으로 표시하세요. iPhone에서 HTML 미리보기가 비어 보이면 [체커보드 PNG](assets/iphone17-pro-max-checkerboard.png)를 사진 앱에서 가로로 엽니다.
2. 가로 9칸의 전체 길이를 자로 재고, 밀리미터로 바꿔 9로 나눕니다. 가로 90 mm이면 한 칸은 **10 mm**, 가로 225 mm이면 **25 mm**입니다. 패턴 크기는 촬영이 끝날 때까지 유지합니다. 패턴은 9×7칸이며, 검출할 내부 교차점은 8×6개입니다.
3. 양쪽 카메라에 전체 무늬와 흰 여백이 들어오게 놓습니다. **노트북**에서 아래 명령을 실행한 뒤, 30초 동안 정면·비스듬한 각도·가까운 위치·먼 위치를 번갈아 보여줍니다. 각 자세에서 2~3초씩 멈춥니다. 두 카메라를 각각 돌리면 보정할 장착 상태가 바뀌므로 차체 전체나 화면을 움직이세요.

```bash
# 한 칸이 실제로 25 mm인 화면 예시. 10 mm라면 SQUARE_MM=10을 사용하세요.
SQUARE_MM=25 bash record_stereo_video.sh 30
```

`captures/videos/` 아래 새 번호 폴더에 원본 AVI 두 개, 프레임 시각 기록, 자동으로 골라낸 `frame-pairs/`가 들어갑니다. 탈락 원인은 `selection-report.json`에서 확인합니다. 탈락 프레임은 별도 사진으로 저장하지 않습니다.

**노트북**에서 보정합니다. 아래 `촬영번호`를 실제 폴더 이름으로, `106.5`를 실측한 두 렌즈 중심 간격(mm)으로 바꿉니다.

```bash
session='captures/videos/촬영번호/frame-pairs'
/usr/bin/python3 scripts/calibrate_stereo.py "$session" --expected-baseline-mm 106.5
```

결과 폴더의 `report.json`을 확인하세요. 두 보정 YAML이 생성되면 아래처럼 적용합니다. 이미 쓰던 YAML이 있다면 먼저 다른 폴더에 복사해 두세요.

```bash
cp "$session/calibration-result/"{left,right}.yaml calibration/
```

거리만 확인하려면 체커보드가 양쪽에 찍힌 프레임 폴더를 하나 선택해 아래 명령을 사용하세요. 한 칸 크기도 실제 값으로 바꿉니다.

```bash
/usr/bin/python3 scripts/measure_stereo.py "$session/프레임폴더명" \
  --calibration-dir calibration --square-mm 25
```

결과는 그 프레임 폴더의 `distance.json`에 저장됩니다. 왼쪽 렌즈 중심에서 패턴 중심까지의 직선거리와 카메라 전방 깊이는 서로 다른 값입니다. 보정과 다른 거리에서 자로 재고 결과를 비교하세요.

## 4. 실시간 ROS 2 준비

### 노트북에서 한 번만 설치

Ubuntu 24.04에 **ROS 2 Jazzy Desktop**을 설치합니다. 아래 명령은 ROS의 apt 저장소가 등록되고 Jazzy가 설치된 뒤 실행합니다. [ROS 2 Jazzy 공식 Ubuntu 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt install python3-colcon-common-extensions ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-lifecycle-manager
bash install_rtabmap.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select jetbot_stereo
source install/setup.bash
```

`source`는 현재 터미널에 ROS 명령을 등록합니다. 새 터미널에서 ROS 명령을 직접 입력할 때는 설치 폴더에서 `source install/setup.bash`를 다시 실행하세요.

### Nano의 컨테이너 확인

**Nano**에 `ssh ros2`로 들어가 `docker ps -a`를 실행합니다. 기존 `jetbot-mini-ros2` Humble 컨테이너가 있고 호스트 네트워크 및 ROS 도메인 16을 쓰면 그대로 사용합니다. 이 경우 컨테이너에 모터 드라이버가 함께 있을 수 있습니다.

영상만 실험하는 새 장치에는, Docker가 설치되고 일반 계정에서 `docker` 명령이 가능한 경우 다음처럼 **영상 발행용 Humble 컨테이너**를 만들 수 있습니다. 아래는 별도 카메라 발행용 초기 설정이며, 기존 제조사 컨테이너를 사용한 하드웨어 검증과 동일한 시험은 하지 않았습니다. 같은 이름의 기존 컨테이너가 있으면 이 명령을 실행하지 마세요.

```bash
docker run -d --name jetbot-mini-ros2 --network host \
  -e ROS_DOMAIN_ID=16 ros:humble-ros-base-jammy sleep infinity
docker exec jetbot-mini-ros2 python3 -c 'import yaml'
```

마지막 명령이 `ModuleNotFoundError`를 출력하면 Nano에서 `docker exec -u 0 jetbot-mini-ros2 bash -c 'apt-get update && apt-get install -y python3-yaml'`로 컨테이너 의존성을 설치합니다. 영상 발행용 컨테이너에는 바퀴 드라이버가 없습니다. `exit`로 노트북으로 돌아옵니다.

### 노트북에서 Nano로 파일 복사

아래는 **노트북**에서 실행하세요. 필요한 스크립트와 코드, 현재 장착용 보정 YAML만 복사합니다.

```bash
ssh ros2 'mkdir -p ~/jetbot_stereo_camera/{scripts,src/jetbot_stereo,calibration}'
scp scripts/{manage_ros_cameras,stream_stereo_gst}.py ros2:jetbot_stereo_camera/scripts/
scp -r src/jetbot_stereo/jetbot_stereo ros2:jetbot_stereo_camera/src/jetbot_stereo/
scp calibration/{left,right}.yaml ros2:jetbot_stereo_camera/calibration/
bash ros_cameras.sh start
bash ros_cameras.sh status
```

이후 보정을 갱신할 때는 `bash ros_cameras.sh stop`으로 영상 연결을 끄고 위 `scp`를 실행하세요. 기존 컨테이너에 영상 코드 또는 보정본이 복사된 적이 있다면, **Nano**에서도 다음 명령으로 컨테이너의 복사본을 갱신한 뒤 다시 시작합니다.

```bash
docker cp "$HOME/jetbot_stereo_camera/." jetbot-mini-ros2:/opt/jetbot-stereo/
```

노트북에서 카메라를 시작한 뒤 다음 명령을 실행하면 영상 수신을 확인할 수 있습니다. 좌우를 각각 보려면 `rqt_image_view` 창의 드롭다운에서 `/stereo/left/image_rect`, `/stereo/right/image_rect`를 선택합니다.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=16 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 topic hz /stereo/left/image_rect
# 초당 수신 프레임 수가 보이면 Ctrl+C를 누르고 다음 줄을 실행
ros2 run rqt_image_view rqt_image_view
```

## 5. RTAB-Map으로 첫 지도 만들기

**노트북**에서 실행하세요.

```bash
bash start_mapping.sh --queue-size=5
```

`입력 확인 완료`가 나오면 양쪽 정렬 영상과 카메라 정보가 들어온 상태입니다. RTAB-Map 창이 열리면 로봇을 평평한 바닥에 놓고 **손으로** 작게 움직여 보세요. 무늬가 없는 벽만 보기보다 벽 모서리와 가구 같이 구분되는 특징이 함께 들어오게 하면 추적에 도움이 됩니다. 빨간 배경과 함께 위치가 멈추면 움직임을 멈추고 특징이 보였던 방향으로 천천히 되돌려 보세요.

- 매핑 터미널에서 `Ctrl+C`로 종료하면 `maps/jetbot-날짜-시간.db`에 저장됩니다. 화면에 표시된 **지도 저장 경로**를 적어두세요.
- 저장된 지도는 `rtabmap-databaseViewer "maps/실제파일명.db"`로 열 수 있습니다. 실행 파일을 찾지 못하면 프로젝트 폴더에서 `source install/setup.bash`를 실행한 뒤 다시 열어보세요.
- 카메라 송출도 종료하려면 노트북에서 `bash ros_cameras.sh stop`을 실행합니다.
- `--queue-size=5`는 수신과 동기화가 기다리는 메시지 수입니다. 작은 값은 오래된 영상이 쌓이는 정도를 줄입니다.

## 6. 흰 가림판과 기존 지도는 별도 실험

**흰 가림판만 지도에 저장**하려면 일반 RTAB-Map 세션을 종료한 뒤 노트북에서 `bash start_boundary_mapping.sh 9.9 0`을 실행합니다. 인수는 바닥에서 카메라 렌즈 중심까지의 높이(cm)와 아래로 기울어진 각도(도)입니다. 두 값 모두 자신의 장착에 맞춥니다. OpenCV로 위·아래 경계를 찾아 벽을 추정하며, 벽 뒤와 관측하지 못한 영역은 벽 지도에 기록하지 않습니다. RViz의 **Fixed Frame**은 `boundary_map_origin`, **MarkerArray** 토픽은 `/partition/wall_markers`입니다. 출력 `maps/boundary-날짜-시간.json`은 RTAB-Map DB와 형식이 다릅니다.

**이미 가진 지도에서 위치 추정**하려면 지도 YAML과 그 안에 지정된 PGM 또는 PNG를 같은 폴더에 둡니다. 예를 들어 `maps/my-room/map.yaml`로 준비한 뒤 실행합니다.

```bash
JETBOT_MAP_YAML="$PWD/maps/my-room/map.yaml" bash start_reference_localization.sh
```

RViz에서 **2D Pose Estimate**를 선택하세요. 지도에서 카메라가 실제로 있는 위치를 누르고, 카메라가 보고 있는 방향으로 드래그합니다. 추정 위치는 `/amcl_pose`에 표시됩니다. 비슷한 벽이 반복되거나 카메라가 벽 하나만 바라보면 위치를 구분하기 어렵습니다. 자로 표시한 15 cm 이동에서 약 62 cm로 추정된 실험이 있었으므로, 실제 거리와 반복 비교하세요. SDF의 정적 박스형 벽으로 만든 `.world`가 있다면 `python3 scripts/world_to_occupancy.py room.world maps/my-room`으로 변환할 수 있습니다. 출력 폴더는 새 이름을 쓰고, 생성된 YAML 이름은 결과에서 확인하세요.

## 7. 자주 막히는 곳

| 증상 | 확인할 것 |
|---|---|
| `ssh: Permission denied` | 노트북의 `~/.ssh/config`에 저장한 JetBot 사용자 이름과 현재 IP를 확인합니다. |
| 사진 한쪽이 없거나 검정 화면 | 다른 촬영 세션을 먼저 종료합니다. Nano 전원을 끈 뒤 CSI 리본 케이블과 센서 장착을 확인합니다. |
| `receiver did not open port 8765` | Nano에서 `docker exec jetbot-mini-ros2 tail -n 40 /tmp/jetbot-stereo-publisher.log`로 보정 YAML, ROS 설치 경로, 수신 포트의 에러를 읽습니다. |
| `카메라 입력이 준비되지 않았습니다` | 노트북에서 `bash ros_cameras.sh status`를 실행합니다. Nano의 발행 수가 늘어도 노트북 변환 PID가 없다면 `tail -n 40 /tmp/jetbot-stereo-decoder.log`를 확인합니다. 양쪽 ROS 도메인은 16입니다. |
| RViz에 지도가 없음 | 현재 실행한 실험이 참조 지도인지 벽 지도인지 확인하고, RViz의 **Fixed Frame**, 지도 토픽 및 **Displays → Status**를 봅니다. |
| `Motor battery missing/low` | 기존 모터 드라이버가 있는 경우 `bash jetbot_drive.sh status`로 전압과 `fault`를 확인합니다. 보호 설정을 유지하고 영상 작업은 손으로 움직이며 계속할 수 있습니다. |

## 실험 결과와 코드 위치

- 스테레오 촬영, 화면 체커보드 보정, 영상 수신, RTAB-Map DB 저장을 실기에서 확인했습니다. 최종 결과는 카메라 장착과 조명, 특징점, 좌우 노출 시차에 따라 달라집니다. 절대 거리 정확도와 장거리 지도 정확도는 검증이 남아 있습니다.
- 기존 모터 보드는 Nano 재부팅 후 정지 상태에서도 전압이 **11.31 V → 1.33 V**로 보고돼 구동을 차단했습니다. 당시 실제 배터리 단자 전압은 확인하지 못했습니다. 이 저장소는 기존 모터 드라이버 설치 파일과 펌웨어 대신 연결 스크립트 `jetbot_drive.sh`를 제공합니다.
- [상세 실험 기록](docs/technical-notes.md)에는 RTAB-Map 설정, 보정 비교, 회전 촬영, 흰 가림판 검출과 위치 추정 실험이 있습니다. [UNO·IMU 재설계 초안](docs/jetbot-mini-redesign-20261119.md)은 기존 보드 교체를 검토한 설계이며 실기 적용 전입니다.
- `src/jetbot_stereo/`는 ROS 패키지, `scripts/`는 촬영·보정·분석 도구, `assets/`는 체커보드와 RViz 설정, `tests/`는 로봇 없이 실행할 수 있는 검증 코드입니다. 실험 공간 사진·원본 동영상·실내 지도·장치 로그는 작업 PC에 보관합니다.

테스트를 실행하려면 노트북 프로젝트 폴더에서 다음을 입력하세요.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="src/jetbot_stereo:${PYTHONPATH:-}" /usr/bin/python3 -m unittest discover -s tests
```
