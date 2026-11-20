#!/usr/bin/env bash
# Guided capture on the laptop; no ROS installation needed on the JetBot.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo '사용법: bash calibrate_stereo.sh [SSH대상]'
    echo '20가지 자세에서 Enter를 눌러 촬영하고 자동 보정합니다. 기본 대상: ros2'
    echo '기본 한 칸 크기: 25 mm. 변경 예: SQUARE_MM=17.285714 bash calibrate_stereo.sh'
    exit 0
fi
target="${1:-ros2}"
if (( $# > 1 )) || [[ "$target" == -* ]]; then
    echo 'SSH 대상 하나만 지정하세요. 예: ros2 또는 ros2@192.0.2.10' >&2
    exit 2
fi
python=/usr/bin/python3
"$python" -c 'import cv2, numpy, yaml' || { echo 'PC에 python3-opencv, python3-numpy, python3-yaml이 필요합니다.' >&2; exit 1; }
square_mm="${SQUARE_MM:-25}"
"$python" -c 'import sys,math; v=float(sys.argv[1]); sys.exit(0 if math.isfinite(v) and v>0 else 1)' "$square_mm" || { echo 'SQUARE_MM는 양수여야 합니다.' >&2; exit 2; }
echo "화면 패턴 크기를 고정하세요. 한 칸 $square_mm mm로 계산합니다."
echo '두 카메라 장착 위치는 고정하고, 로봇 전체 또는 패턴 화면의 자세만 바꾸세요.'
echo '매번 패턴 전체와 흰 여백이 양쪽에 보이게 하고, 촬영 중에는 멈춰주세요.'
echo '기존 Micro-USB 전원에서 재부팅이 계속된다면 먼저 전원을 안정화하세요.'
read -r -p '준비되면 Enter, 취소는 Ctrl+C: ' _
control_dir="$(mktemp -d /tmp/jetbot-calibration-ssh.XXXXXX)"
export JETBOT_SSH_CONTROL_PATH="$control_dir/socket"
cleanup() {
    ssh -S "$JETBOT_SSH_CONTROL_PATH" -O exit "$target" >/dev/null 2>&1 || true
    rm -rf -- "$control_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ssh -M -S "$JETBOT_SSH_CONTROL_PATH" -o ControlPersist=600 -o ConnectTimeout=10 -fnNT "$target"
mkdir -p "$project_dir/captures/calibration-sessions"
session="$(mktemp -d "$project_dir/captures/calibration-sessions/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
"$python" -c 'import json,sys,pathlib; pathlib.Path(sys.argv[1]).write_text(json.dumps({"square_mm":float(sys.argv[2]),"inner_corners":[8,6]},indent=2)+"\n")' "$session/capture-config.json" "$square_mm"
echo "이번 촬영 저장 위치: $session"
poses=(
 '정면, 패턴이 사진 너비의 약 절반'
 '정면, 앞보다 조금 가까이 (패턴 전체 유지)'
 '정면, 앞보다 조금 멀리'
 '패턴이 두 영상의 왼쪽 쪽에 보이게 이동'
 '패턴이 두 영상의 오른쪽 쪽에 보이게 이동'
 '패턴이 두 영상의 위쪽에 보이게 이동'
 '패턴이 두 영상의 아래쪽에 보이게 이동'
 '화면을 왼쪽으로 약 15도 틀어 비스듬히 촬영'
 '화면을 오른쪽으로 약 15도 틀어 촬영'
 '화면 윗부분을 뒤로 약 15도 기울여 촬영'
 '화면 윗부분을 앞으로 약 15도 기울여 촬영'
 '화면을 왼쪽으로 틀고, 조금 가까이'
 '화면을 오른쪽으로 틀고, 조금 가까이'
 '화면 윗부분을 뒤로 기울이고, 조금 멀리'
 '다른 거리에서 화면 윗부분을 앞으로 기울이기'
 '패턴 왼쪽 위 위치, 약간 비스듬히'
 '패턴 오른쪽 위 위치, 약간 비스듬히'
 '패턴 왼쪽 아래 위치, 약간 비스듬히'
 '패턴 오른쪽 아래 위치, 약간 비스듬히'
 '마지막: 지금까지와 다른 거리와 기울기'
)
for i in "${!poses[@]}"; do
    while true; do
        echo
        echo "[$((i+1))/20] ${poses[$i]}"
        read -r -p '자세를 바꾸고 멈춘 뒤 Enter (이 자세 건너뛰기: s): ' answer
        if [[ "$answer" == s ]]; then break; fi
        bash "$project_dir/take_photos.sh" "$target" "$session"
        if "$python" "$project_dir/scripts/calibrate_stereo.py" "$session" --check-only --require-latest; then
            break
        fi
        echo '방금 사진은 패턴이 잘렸거나 이전 자세와 너무 비슷합니다. 조정 후 다시 촬영하세요.'
    done
done
echo '촬영 완료. 이제 보정값을 계산합니다.'
"$python" "$project_dir/scripts/calibrate_stereo.py" "$session" --square-mm "$square_mm"
latest_pair="$("$python" -c 'import pathlib,sys; print(sorted(pathlib.Path(sys.argv[1]).glob("*/camera-0.jpg"))[-1].parent)' "$session")"
"$python" "$project_dir/scripts/measure_stereo.py" "$latest_pair" \
    --calibration-dir "$session/calibration-result" --square-mm "$square_mm"
