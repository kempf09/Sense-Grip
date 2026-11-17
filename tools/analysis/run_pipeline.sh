#!/usr/bin/env bash
# Whole slip-tuning pipeline: tune (grid search through the replay tool) -> plots -> report.
#
#   tools/analysis/run_pipeline.sh <data_dir> <out_dir> [tune_slip.py options...]
#   tools/analysis/run_pipeline.sh --synthetic [<out_dir>] [tune_slip.py options...]
#
# Replay tool: tools/slip_replay/slip_replay when built (make -C tools/slip_replay), else the Python stand-in.
# Override with e.g. --replay python  or  --replay "python3 tools/analysis/replay_py.py".
# --synthetic generates synthetic recordings into a temp dir first: a plumbing test, never tuned thresholds.
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"

usage() { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage

if [ "$1" = "--synthetic" ]; then
  shift
  OUT="${1:-}"
  if [ -n "$OUT" ] && [ "${OUT#--}" = "$OUT" ]; then shift; else OUT="$(mktemp -d -t sg_synth_out.XXXXXX)"; fi
  DATA="$(mktemp -d -t sg_synth_data.XXXXXX)"
  echo "[pipeline] synthetic data -> $DATA"
  "$PY" "$HERE/make_synthetic.py" "$DATA"
else
  [ $# -ge 2 ] || usage
  DATA="$1"; OUT="$2"; shift 2
fi

REPLAY_ARGS=()
REST=()
while [ $# -gt 0 ]; do
  case "$1" in
    --replay) REPLAY_ARGS=(--replay "$2"); shift 2 ;;
    *) REST+=("$1"); shift ;;
  esac
done

mkdir -p "$OUT"
T0=$(date +%s)
"$PY" "$HERE/tune_slip.py" --data "$DATA" --out "$OUT" ${REPLAY_ARGS[@]+"${REPLAY_ARGS[@]}"} ${REST[@]+"${REST[@]}"}
"$PY" "$HERE/plot_recording.py" --data "$DATA" --out "$OUT/plots" --params "$OUT/best_params.json" \
  ${REPLAY_ARGS[@]+"${REPLAY_ARGS[@]}"}
echo "[pipeline] done in $(( $(date +%s) - T0 )) s"
echo "[pipeline] report:      $OUT/report.md"
echo "[pipeline] parameters:  $OUT/best_params.json"
echo "[pipeline] config:      $OUT/config_patch.txt"
echo "[pipeline] plots:       $OUT/plots/"
