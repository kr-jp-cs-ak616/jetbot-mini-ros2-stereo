# JetBot 스테레오 ORB-SLAM2 벽 지도 시험

공식 ORB-SLAM2의 스테레오 추적·로컬 매핑·루프 닫기를 노트북에서 헤드리스로 실행한다. `prepare_stereo.py`가 JetBot의 원본 JPEG 스트림을 촬영 시각으로 짝짓고 기존 보정으로 정렬한다. `map_wall_edges.py`는 추적에 성공한 시각의 영상에서 흰 판의 위·아래 경계를 검출하고, 그 시각의 최적화된 ORB-SLAM2 자세로 벽 구간을 누적한다. 기존 지도에 맞춰 선을 그리는 과정은 없다. 맵의 원점은 첫 카메라 아래 바닥 위치다.

- 카메라의 보정값과 입력 기록의 해상도는 640×480으로 맞춘다. 스테레오 거리 단위에는 YAML 투영행렬에서 계산한 기준선을 사용한다.
- 근거리 벽 면을 만들 때는 해당 녹화의 약 20 cm 높이 판과 3 mm 두께를 가정한다. 추적과 반복 경계 검출이 있는 구간만 결과에 들어간다.
- Nano에서 별도의 카메라 파이프라인을 동시에 열면 기존 ROS 카메라 스트림과 장치 사용이 충돌할 수 있다. 이 시험은 이미 저장한 원본 스트림을 입력으로 사용한다.
- upstream 라이선스는 GPLv3다. 빌드용 소스와 Vocabulary는 `build/` 아래에 둔다. [공식 ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

## 같은 프로그램 빌드

Ubuntu PC 기준. CMake, C++17 컴파일러, OpenCV 개발 라이브러리, Eigen 3, Python의 OpenCV·NumPy·SciPy·PyYAML·Matplotlib가 필요하다. Pangolin 없이 헤드리스로 빌드한다. 아래 명령은 프로젝트 루트에서 실행한다.

```bash
build="$PWD/build/orb-slam2-jetbot"
mkdir -p "$build"
git clone https://github.com/raulmur/ORB_SLAM2.git "$build/upstream"
git -C "$build/upstream" checkout --detach f2e6f51cdc8d067655d90a78c06261378e07e8f3
python3 experiments/orb_slam2/build_headless.py "$build/upstream"
tar -xzf "$build/upstream/Vocabulary/ORBvoc.txt.tar.gz" -C "$build/upstream/Vocabulary"
cmake -S experiments/orb_slam2 -B "$build/headless" \
    -DORB_SLAM2_SOURCE="$build/upstream" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build/headless" --parallel 4
```

## 기존 영상 다시 재생

```bash
bash experiments/orb_slam2/run_replay.sh \
  captures/manual-drive/20261124-163316-id24-id25-first-pass-MJ8Crh \
  maps/rtabmap-aruco-3d-20261125/calibration-recording-candidate \
  maps/orb-slam2-repeat 30
```

PNG에는 궤적과 관측된 벽 구간이 표시된다. PLY는 모델링한 벽 면, JSON은 벽 선 좌표와 프레임 기준, TUM은 ORB-SLAM2 자세, TSV는 각 프레임의 추적 상태를 담는다.

## 저장 결과를 RViz에서 보기

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 experiments/orb_slam2/publish_wall_map.py maps/orb-slam2-20261125/full-rate
```

RViz에서 **Fixed Frame**을 `orb_slam2_local_level`로 지정한다. **Add → MarkerArray**를 누르고 토픽에서 `/orb_slam2/replay_map`을 선택한다. 늦게 열었을 때는 표시 항목의 Durability를 **Transient Local**로 지정한다.
