#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$here/../.." && pwd)"
build="$root/build/orb-slam2-jetbot"
if (( $# < 3 || $# > 4 )); then
    echo 'Usage: run_replay.sh RECORDING CALIBRATION OUTPUT [MIN_FRAME_GAP_MS]' >&2
    exit 2
fi
recording="$1"
calibration="$2"
output="$3"
if [[ -e "$output" ]]; then
    echo "Choose an unused output path: $output" >&2
    exit 2
fi
runner="$build/headless/jetbot_orb_replay"
vocab="$build/upstream/Vocabulary/ORBvoc.txt"
if [[ ! -x "$runner" || ! -r "$vocab" ]]; then
    echo 'Build the upstream headless executable and extract ORBvoc.txt first.' >&2
    exit 2
fi
mkdir -p -- "$output"
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 "$here/prepare_stereo.py" "$recording" "$calibration" "$output/sequence" --min-frame-gap-ms "${4:-99}"
"$runner" "$vocab" "$output/sequence/settings.yaml" "$output/sequence" "$output" | tee "$output/replay.log"
/usr/bin/python3 "$here/map_wall_edges.py" "$output/sequence" "$output"
