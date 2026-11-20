#!/usr/bin/env bash
# New mounting: record raw videos and select frames without applying old calibration.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export JETBOT_SSH_CONTROL_PATH="${JETBOT_SSH_CONTROL_PATH:-/tmp/jetbot-camera-check}"
export SQUARE_MM=25 FRAME_STRIDE=5 MEASURE_AFTER_RECORDING=0
echo '새 장착 보정용: 25 mm 패턴, 렌즈 중심 간격 실측 약 106~107 mm.'
echo '기존 보정으로 거리를 계산하지 않습니다. 움직인 뒤 각 자세에서 3~5초 멈춰주세요.'
exec bash "$project_dir/record_stereo_video.sh" "${1:-60}" "${2:-ros2}"
