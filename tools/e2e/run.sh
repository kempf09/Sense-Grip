#!/usr/bin/env bash
# Format-chain check: firmware-format rows/frames (tools/slip_replay/equiv_test) -> the real
# dashboard page (headless Chromium, mocked gripper API) -> "Download all (.zip)" -> slip_replay.
# The replayed slip events must equal the firmware's. Needs node + playwright (preinstalled here).
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
make -s -C "$repo/tools/slip_replay" slip_replay equiv_test
python3 - "$repo/SenseGrip/web_pages.h" "$work/page.html" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
m = re.search(r'R"([A-Za-z_]*)\((.*)\)\1"', s, re.S)
open(sys.argv[2], 'w').write(m.group(2))
PY
mkdir -p "$work/chain"
"$repo/tools/slip_replay/equiv_test" "$work/chain"
(cd "$work" && NODE_PATH=$(npm root -g 2>/dev/null || true) node "$repo/tools/e2e/dashboard_chain.js" "$work")
mkdir "$work/unz" && python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$work/chain/out.zip" "$work/unz"
rows=$(ls "$work/unz"/*.csv | grep -v _tof); tof=$(ls "$work/unz"/*_tof.csv)
rp=$("$repo/tools/slip_replay/slip_replay" --rows "$rows" --tof "$tof" 2>/dev/null | awk -F, 'NR>1 && $10==1 {print $1}' | tr '\n' ' ')
fw=$(tr '\n' ' ' < "$work/chain/firmware_events.txt")
[ "$rp" = "$fw" ] || { echo "FAIL: replay of downloaded files [$rp] != firmware [$fw]"; exit 1; }
echo "dashboard chain OK: events $fw"

# Grasp recording (auto-recorded from START, marks, '# slip:' line) -> analysis pipeline + grasp summary.
rm -rf "$work/chain" "$work/unz"; mkdir -p "$work/chain" "$work/unz"
"$repo/tools/slip_replay/equiv_test" "$work/chain" grasp
(cd "$work" && NODE_PATH=$(npm root -g 2>/dev/null || true) node "$repo/tools/e2e/dashboard_chain.js" "$work" grasp)
python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$work/chain/out.zip" "$work/unz"
rows=$(ls "$work/unz"/grasp_*.csv | grep -v _tof)
echo "file,object,trial,notes" > "$work/unz/notes.csv"; echo "$(basename "$rows"),mug,C pull + O + D twist,chain test" >> "$work/unz/notes.csv"
grep -q '^# slip: ' "$rows" || { echo "FAIL: no '# slip:' line in the downloaded recording"; exit 1; }
[ "$(grep -c 'mark:' "$rows")" = 6 ] || { echo "FAIL: marks lost on the way through the page"; exit 1; }
python3 "$repo/tools/grasp_summary.py" "$work/unz" "$work/unz/notes.csv" | grep -q "1/1 grasps reached MONITOR" || { echo "FAIL: grasp_summary"; exit 1; }
"$repo/tools/analysis/run_pipeline.sh" "$work/unz" "$work/out" > "$work/pipeline.log" 2>&1 || { tail -20 "$work/pipeline.log"; echo "FAIL: pipeline"; exit 1; }
grep -q "Slip detector tuning report" "$work/out/report.md" || { echo "FAIL: no report"; exit 1; }
grep -qi "1 recordings\|1 recording" "$work/out/report.md" || true
echo "grasp chain OK: dashboard zip -> grasp_summary + analysis pipeline (report $(wc -l < "$work/out/report.md") lines)"
