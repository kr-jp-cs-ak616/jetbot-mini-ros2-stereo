#!/usr/bin/env bash
# Run on the Ubuntu 24.04 laptop, not inside the JetBot Humble container.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
    echo '이 스크립트는 노트북의 Ubuntu 24.04 / ROS 2 Jazzy용입니다.' >&2
    exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '노트북에 ROS 2 Jazzy가 먼저 설치되어 있어야 합니다.' >&2
    exit 1
fi
echo '노트북에 RTAB-Map과 image_proc를 설치합니다.'
if (( EUID == 0 )); then
    admin=()
else
    sudo -v
    admin=(sudo)
fi
"${admin[@]}" apt-get update
"${admin[@]}" apt-get install -y --no-remove ros-jazzy-rtabmap-ros ros-jazzy-image-proc
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$project_dir/log"
report="$project_dir/log/rtabmap-install-verification.txt"
{
    date -Is
    dpkg-query -W ros-jazzy-rtabmap-ros ros-jazzy-image-proc
    for package in rtabmap_launch rtabmap_odom rtabmap_slam rtabmap_viz image_proc; do
        ros2 pkg prefix "$package"
    done
    ros2 pkg executables rtabmap_odom
    ros2 pkg executables rtabmap_slam
    ros2 pkg executables image_proc
} | tee "$report"
echo "설치·패키지 확인 완료. 기록: $report"
echo '이 스크립트는 카메라·모터·지도 생성 노드를 시작하지 않습니다.'
