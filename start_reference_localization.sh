#!/usr/bin/env bash
# Locate the camera relative to the fixed room map.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
map_yaml="${JETBOT_MAP_YAML:-$project_dir/maps/map2-room-reference-20261118/map2_room.yaml}"
if [[ ! -r "$map_yaml" ]]; then
    printf '지도 YAML을 읽을 수 없습니다. JETBOT_MAP_YAML에 파일 경로를 지정하세요. %s\n' "$map_yaml" >&2
    exit 1
fi
map_yaml="$(realpath -- "$map_yaml")"
if pgrep -f '^(/usr/bin/python3 )?[^ ]*/ros2 launch jetbot_stereo (partition|stereo|reference_localization)\.launch\.py' >/dev/null; then
    echo '기존 매핑 또는 위치 추정 세션을 먼저 종료해주세요.' >&2
    exit 1
fi
set +u
source "$project_dir/install/setup.bash"
set -u
export ROS_DOMAIN_ID=16 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
bash "$project_dir/ros_cameras.sh" start
exec ros2 launch jetbot_stereo reference_localization.launch.py \
    map_yaml:="$map_yaml" calibration_dir:="$project_dir/calibration" \
    camera_height_m:=0.099 camera_pitch_deg:="${JETBOT_CAMERA_PITCH_DEG:-3.5}" partition_height_m:=0.199 \
    partition_pitch_tolerance_deg:=3.0 min_panel_luma:=100.0 \
    show_view:=true rviz_config:="$project_dir/assets/reference-map.rviz"
