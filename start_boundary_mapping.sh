#!/usr/bin/env bash
# Preprocessed temporary tracking + boundary masks -> sparse wall-only map.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo '사용법: bash start_boundary_mapping.sh [카메라높이cm=9.9] [하향기울기deg=0]'
    echo '모터를 움직이지 않습니다. 기존 RTAB-Map 매핑은 먼저 종료하세요.'
    exit 0
fi
height_cm="${1:-9.9}";pitch="${2:-0}"
height_m=$(python3 - "$height_cm" "$pitch" <<'PY'
import sys,math
h,p=map(float,sys.argv[1:]);assert math.isfinite(h) and math.isfinite(p) and 2<=h<=200 and abs(p)<=30,'높이/기울기 범위 확인';print(h/100)
PY
)
set +u
source "$project_dir/install/setup.bash"
set -u
export ROS_DOMAIN_ID=16 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
python3 - <<'PYGUARD'
from pathlib import Path
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=(proc/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if any(part==b'jetbot_stereo' and i+1<len(args) and args[i+1] in (b'stereo.launch.py',b'partition.launch.py') for i,part in enumerate(args)) or any(part.endswith(b'/rtabmap_slam/rtabmap') for part in args):
        raise SystemExit('실행 중인 매핑을 먼저 종료하세요. PID '+proc.name)
PYGUARD
bash "$project_dir/ros_cameras.sh" start
map_file="$project_dir/maps/boundary-$(date +%Y%m%d-%H%M%S).json"
echo "벽 셀 지도 저장: $map_file"
echo "높이: ${height_cm}cm, 하향 기울기 가정: ${pitch}도. 실측 거리 정확도는 미검증입니다."
exec ros2 launch jetbot_stereo partition.launch.py calibration_dir:="$project_dir/calibration" \
    map_file:="$map_file" camera_height_m:="$height_m" camera_pitch_deg:="$pitch" \
    show_view:=true rviz_config:="$project_dir/assets/boundary-wall.rviz"
