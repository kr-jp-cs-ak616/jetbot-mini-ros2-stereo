#!/usr/bin/env bash
# Laptop: control camera capture + ROS publisher + local JPEG decoder.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-status}"
case "$action" in start|stop|status) ;; *) echo '사용법: bash ros_cameras.sh start|stop|status' >&2; exit 2;; esac
target="${JETBOT_SSH_TARGET:-ros2}"
control="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-small-area}"
pidfile=/tmp/jetbot-stereo-decoder.pid
decoder_pid() {
    /usr/bin/python3 - "$pidfile" <<'PY'
import os,sys,subprocess
from pathlib import Path
try:
    result=subprocess.run(['systemctl','--user','show','jetbot-stereo-decoder.service',
                           '--property=MainPID','--value'],capture_output=True,text=True)
    service_pid=result.stdout.strip()
except OSError:
    service_pid=''
for candidate in [service_pid,Path(sys.argv[1]).read_text().strip() if Path(sys.argv[1]).exists() else '']:
    try:
        pid=int(candidate)
        assert pid>0 and 'jetbot_stereo.decode_stereo' in Path('/proc/%s/cmdline'%pid).read_text()
        os.kill(pid,0)
        print(pid)
        break
    except (OSError,ValueError,AssertionError):
        pass
PY
}
ssh -o ControlMaster=auto -o ControlPersist=3600 -o "ControlPath=$control" \
    -o ConnectTimeout=10 "$target" \
    "python3 ~/jetbot_stereo_camera/scripts/manage_ros_cameras.py $action"
pid="$(decoder_pid)"
case "$action" in
 start)
    if [[ -z "$pid" ]]; then
        if [[ ! -f "$project_dir/install/setup.bash" ]]; then
            echo '먼저 프로젝트에서 colcon build를 실행해주세요.' >&2; exit 1
        fi
        # ROS setup scripts may reference unset variables.
        set +u
        source "$project_dir/install/setup.bash"
        set -u
        if systemctl --user show-environment >/dev/null 2>&1; then
            if ! systemctl --user is-active --quiet jetbot-stereo-decoder.service; then
                systemd-run --user --unit=jetbot-stereo-decoder --collect \
                    --property=Restart=on-failure --property=RestartSec=3 \
                    /bin/bash -c '
                        source "$1/install/setup.bash"
                        python3 -c '\''import os; from pathlib import Path; Path("/tmp/jetbot-stereo-decoder.pid").write_text(str(os.getppid()))'\''
                        exec env -u ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=16 \
                            ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
                            ROS_LOG_DIR=/tmp/jetbot-stereo-ros-logs \
                            python3 -u -m jetbot_stereo.decode_stereo \
                            > /tmp/jetbot-stereo-decoder.log 2>&1
                    ' _ "$project_dir"
            fi
        else
            nohup env -u ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=16 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
                ROS_LOG_DIR=/tmp/jetbot-stereo-ros-logs \
                python3 -u -m jetbot_stereo.decode_stereo \
                > /tmp/jetbot-stereo-decoder.log 2>&1 < /dev/null &
            echo $! > "$pidfile"
        fi
    fi
    echo '카메라 연결 시작 완료. 노트북과 컨테이너 ROS_DOMAIN_ID=16'
    ;;
 stop)
    systemctl --user stop jetbot-stereo-decoder.service 2>/dev/null || true
    if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; fi
    echo '노트북 영상 변환 노드 종료 완료'
    ;;
 status)
    echo "노트북 영상 변환 PID: ${pid:-실행 안 됨}"
    ;;
esac
