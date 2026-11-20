#!/usr/bin/env bash
# Run on the laptop. All remote diagnostic commands are read-only.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo '사용법: bash collect_jetbot_logs.sh [SSH대상]'
    echo '기본 대상: ros2. 결과는 프로젝트 diagnostics/에 저장합니다.'
    exit 0
fi
target="${1:-ros2}"
if (( $# > 1 )) || [[ "$target" == -* ]]; then
    echo 'SSH 대상 하나만 지정하세요.' >&2
    exit 2
fi
mkdir -p "$project_dir/diagnostics"
output="$(mktemp "$project_dir/diagnostics/$(date +%Y%m%d-%H%M%S)-XXXXXX.log")"
diagnostic_script="$(cat <<'REMOTE'
export LC_ALL=C
echo '=== TIME / BOOT ==='
date -Is
uptime
cat /proc/sys/kernel/random/boot_id
last -x | head -n 20
echo '=== PLATFORM ==='
cat /etc/nv_tegra_release
tr -d '\0' </proc/device-tree/model
echo
echo '=== AVAILABLE JOURNAL BOOTS ==='
journalctl --list-boots --no-pager
echo '=== PREVIOUS BOOT: LAST 200 MESSAGES ==='
journalctl -b -1 -n 200 --no-pager -o short-monotonic
echo '=== PREVIOUS BOOT: KERNEL ==='
journalctl -k -b -1 -n 150 --no-pager -o short-monotonic
echo '=== CURRENT BOOT: KERNEL ==='
dmesg -T | tail -n 150
echo '=== CURRENT TEMPERATURES (MILLI-CELSIUS) ==='
for zone in /sys/class/thermal/thermal_zone*; do
    [ -r "$zone/type" ] || continue
    printf '%s: ' "$(cat "$zone/type")"
    cat "$zone/temp"
done
echo '=== MEMORY / DISK ==='
free -h
df -h / /var/log
echo '=== JOURNAL STORAGE ==='
ls -ld /var/log/journal /run/log/journal 2>/dev/null
echo '=== PSTORE / PERSISTENT CRASH RECORDS ==='
for record in /sys/fs/pstore/*; do
    [ -f "$record" ] || continue
    echo "--- $record ---"
    head -c 32768 "$record"
    echo
done
echo '=== END ==='
REMOTE
)"
# %q quotes shell code correctly for the JetBot user's bash login shell.
printf -v remote_command 'sudo bash -c %q' "$diagnostic_script"
echo '재부팅 후 JetBot이 켜져 있고 네트워크에 연결된 상태에서 실행하세요.'
echo 'SSH 비밀번호와 JetBot sudo 비밀번호가 요청될 수 있습니다.'
echo "저장 위치: $output"
set +e
ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
    "$target" "$remote_command" | tee "$output"
statuses=("${PIPESTATUS[@]}")
set -e
if (( statuses[0] != 0 || statuses[1] != 0 )); then
    echo "로그 수집이 완료되지 않았습니다. 오류와 부분 기록: $output" >&2
    exit 1
fi
echo "로그 저장 완료: $output"
