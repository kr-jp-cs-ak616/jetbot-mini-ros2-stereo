#!/usr/bin/env bash
# Laptop wrapper around the installed Nano driver; no automatic wheel tests.
set -euo pipefail
action="${1:-status}"
target="${JETBOT_SSH_TARGET:-ros2}"
control="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-small-area}"
ssh_options=(-o ControlMaster=auto -o ControlPersist=3600 -o "ControlPath=$control" -o ConnectTimeout=10)
case "$action" in
    status)
        ssh "${ssh_options[@]}" "$target" \
            '/home/ros2/jetbot_ros2/jetbotctl battery; /home/ros2/jetbot_ros2/jetbotctl ros topic echo /jetbot/status std_msgs/msg/String --once --full-length'
        ;;
    stop)
        ssh "${ssh_options[@]}" "$target" '/home/ros2/jetbot_ros2/jetbotctl halt'
        ;;
    teleop)
        if [[ ! -t 0 ]]; then echo '노트북의 일반 터미널에서 실행하세요.' >&2; exit 2; fi
        echo 'W/S 전진·후진, A/D 회전, Space/X 정지, Q 종료.'
        echo '입력이 끊기면 자동 정지합니다. 실제 속도는 바닥과 PWM에 따라 달라집니다.'
        ssh -t "${ssh_options[@]}" "$target" '/home/ros2/jetbot_ros2/jetbotctl teleop'
        ;;
    --help|-h)
        echo '사용법: bash jetbot_drive.sh status|stop|teleop'
        echo 'teleop를 실행하면 모터가 활성화되며 W/S/A/D 입력에 따라 움직입니다.'
        ;;
    *) echo '사용법: bash jetbot_drive.sh status|stop|teleop' >&2; exit 2;;
esac
