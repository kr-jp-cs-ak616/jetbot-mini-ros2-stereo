# ArUco, 사진 기반 복귀와 Nano CUDA

## 1. 마커 인쇄와 설치

[10 cm·15 cm A4 마커](../assets/aruco-100mm-150mm-a4.pdf)를 엽니다. 약 1 m 거리에서는 두 번째 페이지의 15 cm 마커부터 사용합니다. 첫 페이지는 10 cm, ID 22이고 두 번째 페이지는 15 cm, ID 23입니다. 사전은 둘 다 `DICT_4X4_50`입니다.

1. 인쇄 설정을 **A4 세로, 실제 크기 100%**로 맞춥니다. 페이지에 맞춤 기능을 끕니다.
2. 아래 100 mm 확인선을 자로 잽니다. 검은 테두리의 한쪽 바깥선에서 반대쪽 바깥선까지도 재고 mm로 적어둡니다.
3. 무광 종이를 평평하고 움직이지 않는 표면에 붙입니다. 검은 정사각형 주변에는 흰 여백을 남깁니다. 양쪽 카메라에서 모서리 네 개가 다 보여야 합니다.

[15 cm 한 장 버전](../assets/aruco-4x4-50-id23-150mm-a4.pdf)과 [25 cm 두 장 조립 버전](../assets/aruco-4x4-50-id23-250mm-two-a4.pdf)도 있습니다. 두 파일의 ID가 모두 23이므로 한 장면에 하나만 배치합니다.

새 크기나 ID의 A4 파일은 노트북에서 아래처럼 만듭니다. `26:120`은 ID 26, 한 변 120 mm입니다.

```bash
sudo apt install python3-opencv python3-reportlab
/usr/bin/python3 scripts/generate_aruco_print.py --output /tmp/aruco-custom.pdf 26:120
```

## 2. 보정 파일을 지정해 RViz 열기

[README의 카메라·ROS 설치](../README.md)까지 마치고, 자기 공간의 지도 YAML과 그에 연결된 PGM 또는 PNG를 준비합니다. 두 카메라의 위치가 달라지면 그 장착 상태에 맞게 스테레오 보정을 다시 구해야 합니다.

`calibration/aruco-stereo-review/`는 프로젝트의 IMX219 장착 상태에서 만든 **시험용** 프로파일입니다. 실제 렌즈 중심 간격 106.5 mm를 사용했습니다. 마커를 별도 위치로 옮겼을 때 204쌍 모두 좌우 모서리의 세로 차이가 2 px 이내였습니다. 중앙값은 기존 7.27 px에서 0.29 px로 줄었습니다. 이 결과는 해당 장치와 검증 장면의 영상 정렬 결과입니다.

노트북 프로젝트 폴더에서 아래를 실행합니다. `maps/my-room/map.yaml`을 준비한 지도의 실제 경로로 바꿉니다.

```bash
bash start_calibration_trial.sh stop
JETBOT_MAP_YAML="$PWD/maps/my-room/map.yaml" \
JETBOT_TRIAL_CALIBRATION_DIR="$PWD/calibration/aruco-stereo-review" \
bash start_calibration_trial.sh start
```

이 세션은 Nano에서 도메인 16으로 오는 좌우 영상을 받아 노트북 도메인 97에 표시합니다. Nano 카메라가 이미 실행 중이라면 `JETBOT_TRIAL_REUSE_CAMERA=1`을 추가해 현재 스트림을 재사용할 수 있습니다. RViz 지도에서 **2D Pose Estimate**로 현재 위치를 클릭하고 카메라가 보는 방향으로 드래그합니다.

## 3. 기준 사진 저장과 짧은 복귀 시험

노트북 새 터미널에서 프로젝트 폴더를 연 다음 아래를 실행합니다. 마커 한 변이 실측 150 mm와 다르면 `--marker-mm`를 바꿉니다.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
/usr/bin/python3 scripts/aruco_image_return.py --marker-id 23 --marker-mm 150
```

위 명령은 기준 사진만 저장합니다. 출력에 나온 `diagnostics/` 폴더에서 `reference-left.jpg`와 `reference-right.jpg`를 확인합니다. 두 영상에 마커가 들어오면 실제 시험을 준비합니다. 평평한 바닥에서 전진 경로와 그 경로로 후진해 돌아올 공간을 비우고, `bash jetbot_drive.sh status`를 실행해 전압·오류·정지 상태를 확인합니다. 다른 키보드 조종이나 모터 테스트는 끝낸 상태에서 실행합니다.

```bash
/usr/bin/python3 scripts/aruco_image_return.py --marker-id 23 --marker-mm 150 --execute
```

이 명령은 Nano에 설치된 프로젝트 모터 드라이버와 SSH 연결을 사용하며 **실제로 바퀴를 돌립니다**. 기준 사진을 찍고 0.35초 전진한 다음 두 영상의 ArUco 깊이와 모서리를 다시 비교합니다. 후진 시간은 관측값에 따라 결정하고, 각 동작 뒤 0 PWM과 비활성 상태를 확인합니다. 마커가 사라지거나 양쪽 깊이 변화가 크게 다르면 시험을 종료합니다.

실험에서는 영상 비교가 0.22초 후진을 결정했습니다. 복귀 후 기준의 전방 깊이와 0.8 cm, 마커 모서리와 4.7 px 차이가 났습니다. 깊이는 카메라 추정치이며 바닥에서 재지 않았습니다. 현재 스크립트는 직선 구간을 다룹니다. 경로 계획, 장애물 회피, 지도상 임의 지점으로의 이동은 별도 기능입니다.

## 4. Nano CUDA 검증과 학습용 실측 수집

Jetson Nano의 L4T R32.7.1에서 **CUDA 10.2**, **TensorRT 8.2.1**을 확인했습니다. [CUDA 영상 정렬 코드](../scripts/jetson_cuda_rectify.cu)를 Nano 호스트에서 빌드해 실행했습니다. 코드에는 좌우 보정 파일로 생성한 `float32` x/y 보정 맵과 640×480 BGR 프레임 바이트를 입력합니다.

```bash
/usr/local/cuda/bin/nvcc -O3 -arch=sm_53 -std=c++11 jetson_cuda_rectify.cu -o jetson_cuda_rectify
./jetson_cuda_rectify 640 480 left.map left.raw left.gpu
```

마커 위치가 다른 실제 영상 두 쌍을 반복 입력해 좌우 24프레임을 정렬했습니다. GPU와 CPU OpenCV의 픽셀 차이와 ArUco 모서리 좌표 차이는 표본에서 모두 0이었습니다. 프레임당 GPU 복사와 커널 실행은 8.18~9.10 ms였습니다. JPEG 디코딩, 전송과 ROS 처리 시간을 포함한 측정은 별도로 필요하며, 실시간 RViz 세션은 기존 CPU 영상 정렬을 사용합니다.

학습에는 **서로 다른 실제 위치**에서 잰 값이 필요합니다.

1. 마커를 고정하고, 왼쪽 카메라 렌즈 중심부터 마커 중심까지 줄자를 두 점 사이에 곧게 대고 잽니다.
2. 로봇을 움직이지 않은 상태에서 아래 명령으로 한 위치를 저장합니다. `--pose-id`는 실물 위치마다 다르게 적습니다. `98`은 예시 거리(cm)입니다.
3. 가까운 곳과 먼 곳, 화면 왼쪽과 오른쪽에서 같은 절차를 반복합니다. 모델을 평가할 위치는 학습 위치와 따로 유지합니다. 지도 좌표 전체를 보정하려면 마커의 지도 위치와 방향도 실측해 등록해야 합니다.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
/usr/bin/python3 scripts/record_aruco_range_sample.py \
  --pose-id position-01 --marker-id 23 --marker-mm 150 --distance-cm 98
```

대략 잰 거리라면 `--approximate`, 마커 크기에 인쇄 표기값을 사용했다면 `--nominal-marker-size`를 붙입니다. 결과는 로컬 `captures/aruco-range-training/`에 저장됩니다. 첫 실험에서는 실측 약 98 cm에서 추정 97.66 cm와 좌우 관측 20쌍을 기록했습니다. 이 관측들은 **한 실물 위치의 반복값**입니다. 모델 학습과 TensorRT 추론은 아직 수행하지 않았습니다.

## 5. 주행과 벽 거리에서 확인한 범위

- 배터리 교체 후 모터 보드 약 12.05 V, 오류 없음 상태에서 주행을 재개했습니다. 0.80초 전진에서 RViz는 13.6 cm, 영상 오도메트리는 13.2 cm를 추정했습니다. 같은 시간 후진 뒤 시작점까지 남은 추정 차이는 각각 2.7 cm, 2.2 cm였습니다. 이 주행의 실제 거리는 줄자로 재지 않았습니다.
- 줄자로 잰 벽 거리 97.5 cm에서 기존 가림판 경계 방식은 85.8 cm를 보고했습니다. 높이 적용 방식을 바꾼 실물 비교에서는 오차가 더 커져 해당 변경을 되돌렸습니다. 고정 ArUco 관측은 가림판 깊이와 별개의 입력입니다.
- 공개 저장소에는 코드, 인쇄 PDF, 시험용 보정 행렬과 결과 요약이 있습니다. 원본 영상, 실내 지도, 실측 샘플 및 장치 로그는 실험 PC에 보관합니다.
