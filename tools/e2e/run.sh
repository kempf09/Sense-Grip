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
