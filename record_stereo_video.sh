#!/usr/bin/env bash
# One command on the laptop: record a video pair and transfer timing + logs.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo '사용법: bash record_stereo_video.sh [초=30] [SSH대상=ros2]'
    exit 0
fi
seconds="${1:-30}"
target="${2:-ros2}"
if (( $#>2 )) || [[ ! "$seconds" =~ ^[0-9]+$ ]] || (( 10#$seconds<1 || 10#$seconds>120 )) || [[ "$target" == -* ]]; then
    echo '녹화 시간은 1~120초, SSH 대상은 ros2 같은 호스트 이름을 지정하세요.' >&2; exit 2
fi
seconds=$((10#$seconds))
/usr/bin/python3 -c 'import cv2, numpy, yaml' || { echo 'PC에 python3-opencv, python3-numpy, python3-yaml이 필요합니다.' >&2; exit 1; }
mkdir -p "$project_dir/captures/videos"
output="$(mktemp -d "$project_dir/captures/videos/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
archive="$output/.recording.tar"
trap 'rm -f -- "$archive"' EXIT
options=()
if [[ -n "${JETBOT_SSH_CONTROL_PATH:-}" ]]; then options+=(-S "$JETBOT_SSH_CONTROL_PATH" -o BatchMode=yes); fi
echo "두 카메라 ${seconds}초 동영상 녹화. 저장 위치: $output"
# Only the validated integer is substituted into shell code. All GStreamer
# output is captured remotely so it cannot corrupt the binary tar stream.
remote="set -e
work=\$(mktemp -d /tmp/jetbot-video.XXXXXX)
trap 'rm -rf -- \"\$work\"' EXIT
status=0
python3 - \"\$work\" $seconds >\"\$work/recording.log\" 2>&1 || status=\$?
printf '%s\\n' \"\$status\" >\"\$work/status.txt\"
tar -C \"\$work\" -cf - ."
if ! ssh "${options[@]}" -T -o ConnectTimeout=10 -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 "$target" "$remote" \
    < "$project_dir/scripts/record_stereo_gst.py" > "$archive"; then
    echo 'SSH 녹화/전송 실패' >&2; exit 1
fi
tar -xf "$archive" -C "$output"
if [[ "$(cat "$output/status.txt")" != 0 ]]; then
    tail -n 40 "$output/recording.log" >&2
    echo "녹화 실패. 로그: $output/recording.log" >&2; exit 1
fi
echo '동영상과 프레임 시간 기록을 PC에 저장했습니다.'
echo "$output/camera-0.avi"
echo "$output/camera-1.avi"
/usr/bin/python3 "$project_dir/scripts/extract_stereo_video.py" "$output" \
    --square-mm "${SQUARE_MM:-25}" --stride "${FRAME_STRIDE:-1}" --max-temporal-error-px 0.5
if [[ "${MEASURE_AFTER_RECORDING:-1}" == 1 && -f "$project_dir/calibration/left.yaml" && -f "$project_dir/calibration/right.yaml" ]]; then
    latest_pair="$(/usr/bin/python3 -c 'import pathlib,sys; p=sorted(pathlib.Path(sys.argv[1]).glob("frame-pairs/frame-*/camera-0.jpg")); print(p[-1].parent if p else "")' "$output")"
    if [[ -n "$latest_pair" ]]; then
        if ! /usr/bin/python3 "$project_dir/scripts/measure_stereo.py" "$latest_pair" \
            --calibration-dir "$project_dir/calibration" --square-mm "${SQUARE_MM:-25}"; then
            echo '영상은 저장됐지만 이번 프레임의 거리 검증은 통과하지 못했습니다.' >&2
        fi
    fi
fi
