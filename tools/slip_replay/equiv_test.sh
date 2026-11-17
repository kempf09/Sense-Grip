#!/usr/bin/env bash
# Replay equivalence: the firmware-order simulation and slip_replay must fire on the same rows.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
dir=$(mktemp -d); trap 'rm -rf "$dir"' EXIT
"$here/equiv_test" "$dir"
"$here/slip_replay" --rows "$dir/rec.csv" --tof "$dir/rec_tof.csv" 2>/dev/null | awk -F, 'NR>1 && $10==1 {print $1}' > "$dir/replay_events.txt"
fw=$(tr '\n' ' ' < "$dir/firmware_events.txt"); rp=$(tr '\n' ' ' < "$dir/replay_events.txt")
[ -n "$fw" ] || { echo "FAIL: scenario produced no events"; exit 1; }
[ "$fw" = "$rp" ] || { echo "FAIL: firmware events [$fw] != replay events [$rp]"; exit 1; }
echo "equivalence OK: events at rows $fw"
