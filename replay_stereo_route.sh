#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$project_dir/src/jetbot_stereo${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$project_dir/scripts/replay_stereo_route.py" "$@"
