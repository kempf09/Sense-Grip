# Phase 4 runbook — from the operator's recordings to tuned slip detection

The operator records with `docs/RECORDING_PROTOCOL.md` and sends one zip
(`sensegrip_recordings_<date>.zip`, pairs of `name.csv` + `name_tof.csv`) plus a notes file
(`file,object,trial,notes`). Everything below runs on a host (no gripper needed) and is
deterministic.

## 0. Setup (fresh container)

```bash
pip install -r tools/requirements.txt
make -C tests && make -C tools/slip_replay check
```

## 1. Unpack and sanity-check the data

```bash
mkdir -p data/rec_<date> && python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" <zip> data/rec_<date>
cp <notes file> data/rec_<date>/notes.csv
```
`data/` is git-ignored (tens of MB); keep the zip with the thesis material.

Check before tuning (the report's *Data* section prints most of this):
- firmware build in the `# firmware:` line starts with `phase4-slip` (older builds lack marks in grasps);
- ~100 rows/s, frames present (`_tof.csv`), gaps listed;
- marks present (`mark:` events) — **no marks = nothing to tune**; ask the operator;
- recordings with no MONITOR (manual) are listed separately and replayed with `--force-active`;
- the `# slip:` line shows the settings the firmware ran with (all defaults expected).

## 2. Run the pipeline

```bash
tools/analysis/run_pipeline.sh data/rec_<date> out/rec_<date>
```
Outputs `out/rec_<date>/report.md`, `best_params.json`, `config_patch.txt`,
`results_all_sets.csv`, `plots/` (one PNG per recording + one per object). Options:
`--max-fa-per-min` (default 0.5), `--max-dist-frac` (0.3). Details: `tools/analysis/README.md`.

## 3. Read the report, decide

1. Look at the plots of a few recordings per object first (forces, marks, frames features):
   does `fsr_drop` / `tof_zones` react at the marks? Are there obvious artefacts (pad dropouts,
   sensor offline, hand in view)?
2. Overall and per-object detection, latency, false alarms (quiet and disturbances).
3. **Leave-one-object-out** spread: that is the honest estimate for "other objects".
4. Grid-edge warnings: widen `FULL_GRID` in `tools/analysis/tune_slip.py` and re-run if the
   chosen value sits on an edge.
5. Firmware-logged vs replayed events: all should match (1–5 rows late). Any mismatch means the
   firmware and replay disagree — stop and investigate (the CI equivalence test should make this
   impossible).
6. If no parameter set meets the limits: report the trade-off curve (`results_all_sets.csv`) to
   the operator rather than relaxing the limits silently.

## 4. Apply the tuned values (rehearsed 2026-11-18 on a throwaway branch: ~15 lines, all checks green)

1. `SenseGrip/slip_core.cpp` `slipDefaults()`: set the chosen values (config defaults copy them).
2. `SenseGrip/config.cpp`: `CFG_VERSION` 9 → 10, and add `ver == 9 ? sizeof(GripConfig)` to both
   the `stored` and `keep` lists (the struct did not change, so a v9 blob is the full struct).
3. `SenseGrip/config_core.cpp` `upgradeConfig`: `if (fromVersion < 10)` set each slip field that
   still equals its **v9 placeholder** (0.30 / 300 / 0.15 / 30 / 350 / 12 / 0.10 / 4 / 150 / 0 / 2 /
   500 / 500 / 1500) to the new default; values the operator changed are kept. Add a test like
   `config_upgrade_v9_placeholders_take_tuned_defaults` (placeholder → new value, other value kept).
4. `tests/test_slip.cpp`: only `slip_defaults` checks the defaults (the behaviour tests use the
   fixed `P0()` placeholders on purpose) — update its expected values.
5. `make -C tests`, `make -C tools/slip_replay check` (the equivalence rows change with the
   defaults, which is fine: it compares firmware-order vs replay, both on the new defaults),
   `tools/e2e/run.sh`.
6. Re-run the pipeline on the real data as a check; document the numbers in the spec (§4
   placeholders → tuned, with the report's figures) and README "Slip detection".
7. Commit, push, CI green. The operator flashes and runs checklist steps 37–38 (slip warning,
   then **Tighten on slip** on one object).

## 5. Phase 3 checks from the same recordings

```bash
python3 tools/grasp_summary.py data/rec_<date> data/rec_<date>/notes.csv > out/rec_<date>/grasp_summary.md
```
The grasp recordings also show whether phase 3 worked on the gripper: per recording, the
`state:` events (START → PRE_TOUCH → GRASP → MONITOR), `step` (approach → slow), the class, the
force at `grasp_reached`, FAULT reasons. Summarise per object for the operator (which grasps
reached MONITOR, approach stand-off, time per phase) and fix anything that blocked grasps
before tuning further.
