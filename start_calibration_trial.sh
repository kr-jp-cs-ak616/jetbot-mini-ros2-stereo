#!/usr/bin/env bash
# Laptop entry point for the isolated RViz reference-localization trial.
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bridge=jetbot-calibration-bridge.service
localization=jetbot-calibration-localization.service
rviz=jetbot-calibration-rviz.service
calibration="${JETBOT_TRIAL_CALIBRATION_DIR:-$project_dir/calibration/trial-106mm}"
map="${JETBOT_MAP_YAML:-$project_dir/maps/map2-room-reference-20261118/map2_room.yaml}"
target="${JETBOT_SSH_TARGET:-ros2}"
socket="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-small-area}"
pitch="${JETBOT_CAMERA_PITCH_DEG:-3.5}"

stop_trial() {
    for unit in "$localization" "$rviz" "$bridge"; do
        if systemctl --user cat "$unit" >/dev/null 2>&1; then
            systemctl --user stop "$unit"
        fi
    done
}

case "${1:-start}" in
    stop)
        stop_trial
        echo '새 보정 시험용 RViz와 위치 추정을 종료했습니다.'
        exit 0
        ;;
    status)
        for unit in "$bridge" "$localization" "$rviz"; do
            printf '%s %s\n' "$unit" "$(systemctl --user is-active "$unit" || true)"
        done
        exit 0
        ;;
    --help|-h)
        echo '사용법  bash start_calibration_trial.sh [start|status|stop]'
        exit 0
        ;;
    start) ;;
    *) echo '사용법  bash start_calibration_trial.sh [start|status|stop]' >&2; exit 2 ;;
esac

if systemctl --user is-active --quiet "$bridge" &&
   systemctl --user is-active --quiet "$localization"; then
    echo '새 보정 시험이 이미 실행 중입니다. 열려 있는 시험용 RViz를 사용하세요.'
    exit 0
fi

for file in "$project_dir/install/setup.bash" "$calibration/left.yaml" "$calibration/right.yaml" "$map"; do
    if [[ ! -r "$file" ]]; then
        printf '필요한 파일을 읽을 수 없습니다. %s\n' "$file" >&2
        exit 1
    fi
done
calibration="$(realpath -- "$calibration")"
map="$(realpath -- "$map")"

if [[ "${JETBOT_TRIAL_REUSE_CAMERA:-0}" != 1 ]]; then
    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 -S "$socket" "$target" true; then
        echo '노트북 터미널에서 Nano SSH 연결을 먼저 열어주세요.' >&2
        printf '  ssh -M -S %q -o ControlPersist=3600 -fnNT %q\n' "$socket" "$target" >&2
        exit 1
    fi
    ssh -o BatchMode=yes -o ConnectTimeout=5 -S "$socket" "$target" \
        'python3 ~/jetbot_stereo_camera/scripts/manage_ros_cameras.py start'
fi

stop_trial
if ! systemd-run --user --unit="$bridge" --collect --working-directory="$project_dir" \
    /bin/bash -c '
        source "$1/install/setup.bash"
        export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_LOG_DIR=/tmp/jetbot-calibration-trial
        unset ROS_LOCALHOST_ONLY
        exec python3 -u "$1/scripts/relay_trial_cameras.py" "$2"
    ' _ "$project_dir" "$calibration"; then
    exit 1
fi
if ! systemd-run --user --unit="$localization" --collect --working-directory="$project_dir" \
    /bin/bash -c '
        source "$1/install/setup.bash"
        export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
        export ROS_LOG_DIR=/tmp/jetbot-calibration-trial
        unset ROS_LOCALHOST_ONLY
        exec ros2 launch jetbot_stereo reference_localization.launch.py \
            map_yaml:="$3" calibration_dir:="$2" camera_height_m:=0.099 \
            camera_pitch_deg:="$4" partition_height_m:=0.203 \
            partition_pitch_tolerance_deg:=3.0 min_panel_luma:=100.0 \
            show_view:=true rviz_config:="$1/assets/reference-map.rviz"
    ' _ "$project_dir" "$calibration" "$map" "$pitch"; then
    systemctl --user stop "$bridge"
    exit 1
fi

echo '새 보정 시험용 RViz를 시작했습니다. 지도가 보이면 2D Pose Estimate를 사용하세요.'
