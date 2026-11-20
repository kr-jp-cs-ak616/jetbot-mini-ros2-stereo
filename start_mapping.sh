#!/usr/bin/env bash
# Laptop: start existing camera bridge, verify live inputs, launch stereo SLAM.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
viz=true
start_cameras=true
database=''
queue_size=5
for arg in "$@"; do
    case "$arg" in
        --help|-h)
            echo '사용법: bash start_mapping.sh [지도DB경로] [--no-viz] [--no-camera-start] [--queue-size=5]'
            echo '카메라 연결 후 RTAB-Map을 실행합니다. Ctrl+C로 지도 저장 후 종료합니다.'
            exit 0;;
        --no-viz) viz=false;;
        --no-camera-start) start_cameras=false;;
        --queue-size=*) queue_size="${arg#*=}";;
        --*) echo "알 수 없는 옵션: $arg" >&2; exit 2;;
        *) if [[ -n "$database" ]]; then echo 'DB 경로는 하나만 지정하세요.' >&2; exit 2; fi
           database="$arg";;
    esac
done
if [[ ! "$queue_size" =~ ^[0-9]+$ ]] || (( 10#$queue_size < 1 || 10#$queue_size > 100 )); then
    echo 'queue-size는 1~100 사이 정수여야 합니다.' >&2; exit 2
fi
queue_size=$((10#$queue_size))
if [[ ! -f "$project_dir/install/setup.bash" ]]; then
    echo '먼저 프로젝트에서 colcon build --symlink-install을 실행해주세요.' >&2; exit 1
fi
set +u
source "$project_dir/install/setup.bash"
set -u
ros2 pkg prefix rtabmap_launch >/dev/null
export ROS_DOMAIN_ID=16
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
if [[ "$start_cameras" == true ]]; then
    bash "$project_dir/ros_cameras.sh" start
fi
echo '좌우 정렬 영상과 보정 정보 수신 확인 중 (최대 20초)...'
/usr/bin/python3 - <<'PY'
import time
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
rclpy.init()
node=rclpy.create_node('jetbot_mapping_input_check')
received={}
stamps={side:deque(maxlen=30) for side in ('left','right')}
counts={}
def on_message(msg,key):
    received[key]=msg
    counts[key]=counts.get(key,0)+1
    if key.endswith('/image_rect') and (msg.width,msg.height)==(640,480):
        stamps[key.split('/')[0]].append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
for side in ('left','right'):
    for suffix,msg_type in [('image_rect',Image),('camera_info',CameraInfo)]:
        key=side+'/'+suffix
        node.create_subscription(msg_type,'/stereo/'+key,
                                 lambda msg,k=key:on_message(msg,k),qos_profile_sensor_data)
deadline=time.monotonic()+20
ready=False
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
    if len(received)==4:
        ready=any(abs(left-right)<=.02 for left in stamps['left'] for right in stamps['right'])
        if ready:break
node.destroy_node()
rclpy.shutdown()
if not ready:
    print('토픽별 수신 프레임:',counts)
    if stamps['left'] and stamps['right']:
        print('좌우 최소 시간차(ms):',round(min(abs(l-r) for l in stamps['left'] for r in stamps['right'])*1000,2))
    raise SystemExit('카메라 입력이 준비되지 않았습니다. bash ros_cameras.sh status로 상태를 확인하세요.')
print('입력 확인 완료: 좌우 640×480, 정렬 영상과 CameraInfo 수신')
PY
mkdir -p "$project_dir/maps"
if [[ -z "$database" ]]; then
    database="$project_dir/maps/jetbot-$(date +%Y%m%d-%H%M%S).db"
fi
echo "지도 저장 경로: $database"
echo "수신/동기화 대기열: $queue_size/$queue_size"
echo '모터 명령은 보내지 않습니다. 카메라 두 대의 상대 장착 관계를 유지하세요.'
exec ros2 launch jetbot_stereo stereo.launch.py \
    mapping:=true start_capture:=false start_rectification:=false \
    calibration_dir:="$project_dir/calibration" \
    topic_queue_size:="$queue_size" sync_queue_size:="$queue_size" \
    database_path:="$database" viz:="$viz"
