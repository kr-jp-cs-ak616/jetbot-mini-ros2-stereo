#!/usr/bin/env bash
# Run on the laptop: capture both JetBot CSI cameras over one SSH connection.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    echo "사용법: bash take_photos.sh [SSH대상] [저장폴더]"
    echo "기본 SSH 대상: ros2"
    echo "기본 저장폴더: $script_dir/captures"
    echo '예: bash take_photos.sh ros2'
    exit 0
fi
if (( $# > 2 )); then
    echo '인수는 SSH 대상과 저장폴더 두 개까지 지정할 수 있습니다.' >&2
    exit 2
fi
ssh_target="${1:-ros2}"
output_root="${2:-$script_dir/captures}"
if [[ "$ssh_target" == -* ]]; then
    echo '올바른 SSH 대상(예: ros2 또는 ros2@192.0.2.10)을 지정하세요.' >&2
    exit 2
fi
for program in ssh tar mktemp; do
    command -v "$program" >/dev/null || { echo "필요한 프로그램: $program" >&2; exit 1; }
done
mkdir -p -- "$output_root"
output_root="$(cd -- "$output_root" && pwd)"
output_dir="$(mktemp -d "$output_root/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
archive="$output_dir/.photos.tar"
trap 'rm -f -- "$archive"' EXIT

echo "JetBot 접속: $ssh_target"
echo '두 카메라를 촬영합니다. SSH 비밀번호가 나오면 터미널에 입력하세요.'
echo '촬영이 끝날 때까지 카메라와 촬영 대상을 멈춰주세요.'

ssh_options=()
if [[ -n "${JETBOT_SSH_CONTROL_PATH:-}" ]]; then
    ssh_options+=(-S "$JETBOT_SSH_CONTROL_PATH" -o BatchMode=yes)
fi
if ! ssh "${ssh_options[@]}" -T -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
    "$ssh_target" 'bash -s' > "$archive" <<'REMOTE'
set -euo pipefail
if [[ ! -r /etc/nv_tegra_release ]]; then
    echo '오류: SSH 대상이 JetPack이 설치된 Jetson인지 확인하세요.' >&2
    exit 1
fi
for program in gst-launch-1.0 timeout tar; do
    command -v "$program" >/dev/null || { echo "JetBot에 필요한 프로그램: $program" >&2; exit 1; }
done
work="$(mktemp -d /tmp/jetbot-photos.XXXXXX)"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
    rm -rf -- "$work"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

for id in 0 1; do
    timeout --signal=INT --kill-after=5s 40s \
        gst-launch-1.0 -e nvarguscamerasrc sensor-id="$id" num-buffers=30 \
        ! 'video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12' \
        ! nvvidconv ! 'video/x-raw,format=I420' ! jpegenc \
        ! multifilesink max-files=1 location="$work/camera-$id-%05d.jpg" \
        > "$work/camera-$id.log" 2>&1 &
    pids+=("$!")
done
failed=0
shopt -s nullglob
for id in 0 1; do
    result=0
    wait "${pids[$id]}" || result=$?
    images=("$work/camera-$id-"*.jpg)
    if [[ "$result" -eq 0 && "${#images[@]}" -eq 1 && -s "${images[0]}" ]]; then
        mv -- "${images[0]}" "$work/camera-$id.jpg"
        echo "카메라 $id 촬영 성공" >&2
    else
        echo "카메라 $id 촬영 실패 (종료 코드 $result)" >&2
        tail -n 15 "$work/camera-$id.log" >&2
        failed=1
    fi
done
# Both children have been reaped; cleanup must not signal a reused process ID.
pids=()
printf '%s\n' "$failed" > "$work/status.txt"
tar -C "$work" -cf - .
REMOTE
then
    echo 'SSH 접속 또는 사진 전송에 실패했습니다. 위 오류를 확인하세요.' >&2
    exit 1
fi
tar -xf "$archive" -C "$output_dir"
if [[ "$(cat "$output_dir/status.txt")" != 0 ]]; then
    echo "촬영에 실패한 카메라가 있습니다. 로그 저장 위치: $output_dir" >&2
    exit 1
fi
for id in 0 1; do
    if [[ ! -s "$output_dir/camera-$id.jpg" ]]; then
        echo "카메라 $id 사진이 비어 있거나 없습니다: $output_dir" >&2
        exit 1
    fi
done
echo
echo '사진 두 장을 PC에 저장했습니다.'
echo "$output_dir/camera-0.jpg"
echo "$output_dir/camera-1.jpg"
