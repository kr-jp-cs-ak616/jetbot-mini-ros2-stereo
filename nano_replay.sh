#!/usr/bin/env bash
# Install and supervise replay on the JetBot Nano. Drive decisions stay in the container.
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target="${JETBOT_SSH_TARGET:-ros2}"
control="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-marker-run}"
base=/tmp/jetbot-nano-replay
container=jetbot-mini-ros2
ssh_opts=(-S "$control" -o BatchMode=yes -o ConnectTimeout=6)
remote() { ssh "${ssh_opts[@]}" -T "$target" "$@"; }

case "${1:-}" in
    deploy)
        report="${2:?Provide offline-replay-validation.json}"
        wheel="${3:-}"
        stage="$(mktemp -d)"
        trap 'rm -rf -- "$stage"' EXIT
        mkdir -p "$stage"/{scripts,jetbot_stereo,calibration/aruco-stereo-review,anchors,route}
        cp "$project"/scripts/{replay_stereo_route,replay_route_bridge,replay_route_vision,replay_marker_pause,record_aruco_range_sample,aruco_initial_pose}.py "$stage/scripts/"
        cp "$project"/src/jetbot_stereo/jetbot_stereo/{__init__,boundary_walls,calibration}.py "$stage/jetbot_stereo/"
        cp "$project"/calibration/aruco-stereo-review/{left,right}.yaml "$stage/calibration/aruco-stereo-review/"
        cp "$project"/maps/map2-room-reference-20261118/aruco-id{22,23,24,25}-anchor.json "$stage/anchors/"
        /usr/bin/python3 - "$report" "$stage/route" <<'PY'
import json
from pathlib import Path
import shutil
import sys
source, target = map(Path, sys.argv[1:])
report = json.loads(source.read_text())
for step in report['route']:
    label = step['label']
    if not label or Path(label).name != label:
        raise ValueError(f'Invalid route label {label!r}')
    for side in ('left', 'right'):
        file = f'{label}-before-{side}.jpg'
        shutil.copyfile(source.parent / file, target / file)
shutil.copyfile(source, target / 'offline-replay-validation.json')
PY
        if [[ -n "$wheel" ]]; then cp -- "$wheel" "$stage/opencv-aarch64.whl"; fi
        tar -C "$stage" -cf - . | remote "docker exec -i $container /jetbot-entrypoint.sh sh -ec '
            mkdir -p $base
            cd $base
            exec 9>drive.lock
            flock -n 9
            tar -xf -
            if [ -e opencv-aarch64.whl ]; then
                mkdir -p dependencies
                python3 -m zipfile -e opencv-aarch64.whl dependencies
                rm -- opencv-aarch64.whl
            fi
            PYTHONPATH=$base/dependencies:$base:\${PYTHONPATH:-} python3 -c \"import cv2; import rclpy; print(cv2.__version__, cv2.aruco.__name__)\"
        '"
        ;;
    inspect|probe|test|motion-test|patrol|start)
        action="$1"
        detached=0
        goal="${2:-22}"
        if [[ "$goal" != 22 && "$goal" != 25 ]]; then
            printf '%s\n' '지원하는 목표 마커는 22번과 25번입니다.' >&2
            exit 2
        fi
        if [[ "$action" == start ]]; then
            detach=-d
            detached=1
            flag=--execute
        elif [[ "$action" == test ]]; then
            detach=-d
            detached=1
            flag=--inspect
        elif [[ "$action" == motion-test ]]; then
            detach=-d
            detached=1
            flag=--motion-test
        elif [[ "$action" == patrol ]]; then
            detach=-d
            detached=1
            flag=--patrol
        elif [[ "$action" == inspect ]]; then
            detach=
            flag=--inspect
        else
            detach=
            flag=
        fi
        remote "docker exec $detach -e JETBOT_NANO_DETACHED=$detached $container /jetbot-entrypoint.sh sh -ec '
            cd $base
            exec 9>drive.lock
            flock -n 9
            rm -f -- stop.requested
            mkdir -p runs
            if [ \"\$JETBOT_NANO_DETACHED\" = 1 ]; then exec >current.log 2>&1; fi
            export PYTHONPATH=$base/dependencies:$base:\${PYTHONPATH:-}
            exec python3 -u scripts/replay_stereo_route.py route/offline-replay-validation.json \
                --onboard --anchors anchors --output runs/\$(date +%Y%m%d-%H%M%S-%N) \
                --stop-file stop.requested \"\$@\"
        ' _ $flag --goal-id $goal"
        if (( detached )); then echo 'Nano에 시작을 요청했습니다. nano_replay.sh status로 결과를 확인하세요.'; fi
        ;;
    status)
        remote "docker exec $container sh -ec 'cd $base
            if flock -n drive.lock true; then echo idle; else echo running; fi
            if [ -f current.log ]; then tail -25 current.log; fi'"
        ;;
    stop)
        remote "docker exec $container touch $base/stop.requested"
        ;;
    *)
        printf '%s\n' '사용법  nano_replay.sh deploy 기록JSON [ARM64_OpenCV_휠] | inspect | probe | test | motion-test | patrol | start | status | stop'
        exit 2
        ;;
esac
