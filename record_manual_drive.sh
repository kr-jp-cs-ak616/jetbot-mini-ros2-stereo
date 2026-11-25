#!/usr/bin/env bash
# Start Nano-side recording, run the installed keyboard driver, then fetch the take.
set -euo pipefail

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target="${JETBOT_SSH_TARGET:-ros2}"
control="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-small-area}"
output_root="${JETBOT_DRIVE_RECORDINGS:-$project/captures/manual-drive}"
container=jetbot-mini-ros2

usage() {
    cat <<'HELP'
사용법
  bash record_manual_drive.sh [구간이름]
  bash record_manual_drive.sh --check
  bash record_manual_drive.sh --recover 기록ID

구간이름 예시  id23-to-id22, id22-to-id23, roundtrip-01
W/S 전진·후진, A/D 회전, Space/X 정지, Q 종료 및 저장
출발과 도착 때는 마커가 양쪽 화면에 들어간 자세로 2~3초간 정지하세요.
--check는 카메라·모터 상태만 짧게 기록합니다.
--recover는 Nano에 남아 있는 기록을 다시 가져옵니다.
기본 저장 위치는 프로젝트의 captures/manual-drive/ 입니다.
HELP
}

mode=drive
label="${1:-manual-drive}"
case "$label" in
    -h|--help) usage; exit 0 ;;
    --check) mode=check; label=check ;;
    --recover) mode=recover; label="${2:-}" ;;
esac
if [[ ! "$label" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]{0,119}$ || "$target" == -* ]] ||
   { [[ "$mode" == recover ]] && (( $# != 2 )); } ||
   { [[ "$mode" != recover ]] && (( $# > 1 || ${#label} > 80 )); }; then
    usage >&2
    exit 2
fi
if [[ "$mode" == drive && ! -t 0 ]]; then
    echo '키 입력을 받을 수 있는 노트북 터미널에서 실행하세요.' >&2
    exit 2
fi

ssh_opts=(-o "ControlPath=$control" -o ConnectTimeout=8
          -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
if ! ssh "${ssh_opts[@]}" -O check "$target" >/dev/null 2>&1; then
    ssh "${ssh_opts[@]}" -o ControlMaster=auto -o ControlPersist=3600 -MNf "$target"
fi
remote() {
    ssh "${ssh_opts[@]}" -T -o BatchMode=yes "$target" "$@"
}

mkdir -p "$output_root"
if [[ "$mode" == recover ]]; then
    run_id="$label"
    local_dir="$output_root/$run_id"
    mkdir -p "$local_dir"
else
    local_dir="$(mktemp -d "$output_root/$(date +%Y%m%d-%H%M%S)-$label-XXXXXX")"
    run_id="${local_dir##*/}"
fi
remote_dir="/tmp/jetbot-drive-recordings/$run_id"
recording_started=0
teleop_active=0

request_stop() {
    remote "docker exec $container touch '$remote_dir/stop.requested'"
}

wait_for_summary() {
    remote "for i in \$(seq 1 50); do
        if docker exec $container test -s '$remote_dir/summary.json'; then exit 0; fi
        sleep .2
    done
    docker exec $container cat '$remote_dir/recorder.log' >&2 || true
    exit 1"
}

fetch() {
    local staging
    staging="$(mktemp -d "$local_dir/.download-XXXXXX")"
    if remote "docker exec $container tar -C '$remote_dir' -cf - left.mjpeg right.mjpeg events.jsonl summary.json recorder.log" >"$staging/take.tar" &&
       tar -xf "$staging/take.tar" -C "$staging" &&
       [[ -s "$staging/events.jsonl" && -s "$staging/summary.json" ]]; then
        mv -- "$staging"/{left.mjpeg,right.mjpeg,events.jsonl,summary.json,recorder.log} "$local_dir/"
        rm -f -- "$staging/take.tar"
        rmdir -- "$staging"
        return 0
    fi
    return 1
}

cleanup() {
    if (( teleop_active )); then
        remote '/home/ros2/jetbot_ros2/jetbotctl halt' >/dev/null 2>&1 || true
    fi
    if (( recording_started )); then
        request_stop >/dev/null 2>&1 || true
        printf '기록 ID %s. 다시 가져오기: bash "%s" --recover %s\n' "$run_id" "$0" "$run_id" >&2
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$mode" == recover ]]; then
    echo "Nano의 기록을 가져옵니다. 기록 ID $run_id"
    request_stop
else
    remote 'python3 "$HOME/jetbot_stereo_camera/scripts/manage_ros_cameras.py" start'
    remote "set -eu
        upload=\$(mktemp)
        trap 'rm -f -- \"\$upload\"' EXIT
        cat > \"\$upload\"
        chmod 0644 \"\$upload\"
        docker exec $container mkdir -p /tmp/jetbot-drive-recordings
        docker exec $container mkdir '$remote_dir'
        docker cp \"\$upload\" $container:'$remote_dir/recorder.py'
        docker exec -d $container /jetbot-entrypoint.sh bash -c 'exec python3 -u \"\$1/recorder.py\" \"\$1\" > \"\$1/recorder.log\" 2>&1' _ '$remote_dir'" \
        < "$project/scripts/record_manual_drive.py"
    recording_started=1
    echo "Nano에서 기록을 시작합니다. 기록 ID $run_id"
    if ! remote "for i in \$(seq 1 65); do
        if docker exec $container test -s '$remote_dir/ready.json'; then exit 0; fi
        if docker exec $container test -s '$remote_dir/summary.json'; then break; fi
        sleep .2
    done
    docker exec $container cat '$remote_dir/waiting.json' >&2 || true
    docker exec $container cat '$remote_dir/recorder.log' >&2 || true
    exit 1"; then
        echo '카메라 또는 모터 상태를 확인하지 못해 주행을 열지 않았습니다.' >&2
        exit 1
    fi

    if [[ "$mode" == drive ]]; then
        echo '기록 중입니다. 출발 마커 앞에서 2~3초 잠깐 멈춘 뒤 출발하세요.'
        teleop_active=1
        if ! JETBOT_SSH_TARGET="$target" JETBOT_SSH_CONTROL_PATH="$control" \
             bash "$project/jetbot_drive.sh" teleop; then
            echo '키보드 조종이 중단됐습니다. 모터 정지를 요청하고 기록을 가져옵니다.' >&2
            remote '/home/ros2/jetbot_ros2/jetbotctl halt' || true
        fi
        teleop_active=0
        sleep .5
    else
        echo '좌우 영상과 모터 상태가 확인됐습니다. 카메라만 4초 더 기록합니다.'
        sleep 4
    fi
    request_stop
fi

if ! wait_for_summary || ! fetch; then
    echo "전송을 마치지 못했습니다. 다시 가져오기: bash \"$0\" --recover $run_id" >&2
    exit 1
fi
recording_started=0
printf '\n저장 위치\n%s\n' "$local_dir"
python3 - "$local_dir/summary.json" <<'PY'
import json
import sys
from pathlib import Path

data = json.loads(Path(sys.argv[1]).read_text())
frames = data["frames"]
print(f'왼쪽 {frames["left"]}장 · 오른쪽 {frames["right"]}장')
print(f'주행 명령 {data["command_samples"]}개 · 모터 상태 {data["motor_status_samples"]}개')
status = data.get('final_motor_status') or {}
if status.get('pwm') == [0, 0] and status.get('enabled') is False:
    print('기록 마지막 상태에서 모터 정지·비활성화가 확인됐습니다.')
if data['stop_reason'] != 'stop requested':
    print(f'기록 중단 사유  {data["stop_reason"]}', file=sys.stderr)
    sys.exit(1)
PY
