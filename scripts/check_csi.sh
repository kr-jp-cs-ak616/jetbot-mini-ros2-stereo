#!/usr/bin/env bash
# Run on the Jetson host. Capture both CSI cameras concurrently without a GUI.
set -euo pipefail

output_dir="${1:-/tmp/jetbot-camera-check-$(date +%Y%m%d-%H%M%S)}"
capture_width="${WIDTH:-640}"
capture_height="${HEIGHT:-480}"
capture_fps="${FPS:-15}"
left_sensor="${LEFT_SENSOR_ID:-0}"
right_sensor="${RIGHT_SENSOR_ID:-1}"

if [[ ! -r /etc/nv_tegra_release ]]; then
    echo 'Run this script on the Jetson host with JetPack installed.' >&2
    exit 1
fi
for program in gst-launch-1.0 gst-inspect-1.0 timeout; do
    command -v "$program" >/dev/null || { echo "Missing program: $program" >&2; exit 1; }
done
if [[ "$left_sensor" == "$right_sensor" ]]; then
    echo 'Left and right sensor IDs must differ.' >&2
    exit 1
fi
mkdir -p "$output_dir"
cat /etc/nv_tegra_release > "$output_dir/jetpack.txt"
gst-inspect-1.0 nvarguscamerasrc > "$output_dir/argus-plugin.txt"

capture() {
    local side="$1" sensor="$2"
    # Keep the last JPEG after 30 frames, allowing some automatic exposure settling.
    timeout --signal=INT --kill-after=5s 40s \
        gst-launch-1.0 -e nvarguscamerasrc sensor-id="$sensor" num-buffers=30 \
        ! "video/x-raw(memory:NVMM),width=$capture_width,height=$capture_height,framerate=$capture_fps/1,format=NV12" \
        ! nvvidconv ! 'video/x-raw,format=I420' ! jpegenc \
        ! multifilesink max-files=1 location="$output_dir/$side-%05d.jpg" \
        > "$output_dir/$side.log" 2>&1
}

echo "Capturing sensors $left_sensor and $right_sensor concurrently. Output: $output_dir"
echo 'Stop other applications using these cameras before running this check.'
capture left "$left_sensor" &
left_pid=$!
capture right "$right_sensor" &
right_pid=$!
left_status=0
right_status=0
wait "$left_pid" || left_status=$?
wait "$right_pid" || right_status=$?

failed=0
for side in left right; do
    if [[ "$side" == left ]]; then status="$left_status"; else status="$right_status"; fi
    images=("$output_dir/$side-"*.jpg)
    if [[ "$status" -eq 0 && -s "${images[0]}" ]]; then
        echo "PASS $side: ${images[0]}"
    else
        echo "FAIL $side (exit $status). Log: $output_dir/$side.log" >&2
        tail -n 30 "$output_dir/$side.log" >&2
        failed=1
    fi
done
if [[ "$failed" -eq 0 ]]; then
    echo 'Both cameras delivered JPEGs concurrently. Inspect images for orientation, focus and overlapping views.'
    echo 'This check does not prove hardware synchronization or distance accuracy.'
fi
exit "$failed"
