# Phase 4b — Slip detection and tighten — design

Date: 2026-11-17 · Branch: `feature/continuous-approach-touch-window` · Builds on: phase 3, 4a
Status: operator decisions taken 2026-11-17; thresholds to be tuned from the hardware recordings
(docs/RECORDING_PROTOCOL.md). Until then every threshold is a placeholder.

## 1. Why / what the operator decided
- Slip = the held object (1) moving away from the palm, (2) rotating, (3) slipping through the
  fingers. Detect with FSR + SEN0628 fusion (operator, phase-3 interview).
- On slip: dashboard warning + `slip:` event in the recording, always. **Tighten on slip**
  is a switch (operator 2026-11-17): when on, re-close slowly to a higher target force, at most
  2 times per grasp, never above Max force, never in Delicate. When off, warning only, plus a
  manual **Tighten** button.
- Gripper is **palm up** in the demo (gravity pulls the object into the palm, so "away from the
  palm" is an external pull or a tipping object, not a drop).
- Must work on egg, tube, mug, apple, drill handle, the asymmetric object **and other objects**:
  thresholds are relative (fraction of the hold force / of the object range), not per object.
- The operator marks slip onsets (keys P/T/S, O for non-slip disturbances, E for end).

## 2. Goals / non-goals / anti-goals
- **Goals:** detect marked slips within 0.5 s of the mark; ignore quiet holds (TPE relaxation:
  slow force decay) and most "other" disturbances; same code in firmware and offline replay.
- **Non-goals:** force regulation in MONITOR; slip prediction; per-object thresholds.
- **Anti-goals:** any motion caused by the detector while the switch is off; tightening the
  egg; tightening above Max force; a tighten that ends in FAULT just because the object moved
  (a failed tighten returns to MONITOR with a warning).

## 3. Facts the design relies on
- MONITOR = torque off; the jaw is not back-drivable, so **in MONITOR the fingers do not move**:
  any change in the SEN0628 frames is the object or something else in view, never the fingers.
- FSR filtered force per pad at 100 Hz; contacts debounced per finger; force limit uses
  max(raw, filtered).
- SEN0628 frame sets at ~10–20 Hz; range error ±10 % of the reading (mostly systematic per
  zone, so a change against the zone's own baseline is far more precise than an absolute value).

## 4. Detector (`slip_core`, pure, host-tested; the same object code is linked into the replay tool)
Runs only in MONITOR (and is reset on MONITOR entry). After `settleMs` (500 ms) it takes the
**baseline**: per pad force, the raw SEN0628 frame, and the smoothed object measurement.

**FSR feature (fast slips, pulls):** for every pad whose baseline ≥ `fsrMinForceN` (0.15 N):
relative drop = (max force over the last `fsrWindowMs` (300 ms) − current) / baseline. Flag when
the largest pad drop ≥ `fsrDropFrac` (0.30) for `fsrConfirmMs` (30 ms). A pad going from
contact to no contact counts as a drop of 1. The short window ignores slow relaxation.

**ToF feature (moving away, rotating, sliding):** against the baseline frame, count zones that
were valid (1..3999 mm) and ≤ `tofNearMm` (350 mm) at baseline and now differ by more than
max(`tofZoneMm` (12 mm), `tofZoneFrac` (0.10) × baseline). Flag when the count ≥ `tofZoneCount`
(4) on `tofConfirmFrames` (2) consecutive frame sets. Secondary: the smoothed object measurement
(range, height, per-sensor radius) moved by more than max(`tofDispMm`, 10 % of range) —
always reported as a feature; it also sets the ToF flag only when `tofDispMm` > 0 (default 0 = off
until the recordings show it is reliable while the fingers cover the object).

**Fusion (`mode`):** 0 = FSR only, 1 = ToF only, 2 = either (default until tuned), 3 = both
within `fuseWindowMs` (500 ms). A slip **event** fires on the first flagged step, then nothing
for `holdoffMs` (1500 ms). Event argument: which features were flagged (`slip:fsr`, `slip:tof`,
`slip:fsr+tof`).

All parameters live in `GripConfig` (v9), editable on the SEN0628 tab, and are the replay
tool's parameters (same names).

## 5. Tighten
- Config `slipTighten` (bool, **default off**), editable next to the slip settings. RAM switches
  stay as they are (override / delicate / approach only).
- Trigger: slip event in MONITOR with `slipTighten` on, fewer than `tightenMax` (2) tightens
  this grasp, class not DELICATE → FSM goes to GRASP in **tighten mode**: target force =
  min(previous target × `tightenGain` (1.25), 0.9 × Max force), class grasp speed and current.
  Event `tighten:<n>`.
- Manual **Tighten** button (Combined tab, MONITOR only): same, ignores the switch, still
  respects Delicate, the 2-per-grasp limit and the force cap.
- Tighten mode reaches the target → MONITOR, `tighten_done`, detector re-baselines.
  Close limit or GRASP timeout in tighten mode → **MONITOR** (torque off), `tighten_failed`,
  dashboard warning — not FAULT. Force limit, servo faults, E-stop: FAULT as always.

## 6. Dashboard / log
- Combined tab: a red **SLIP** badge with the reason and time since, cleared by Open; the
  Tighten button; tighten count.
- Recording events (already reserved in phase 4a): `slip:...`, `tighten:<n>`, `tighten_done`,
  `tighten_failed`.

## 7. Offline replay and tuning (the process for the hardware data)
- `tools/slip_replay` (host C++, links `SenseGrip/slip_core.cpp`): replays a recording
  (`name.csv` + `name_tof.csv`) through the detector with given parameters and writes per-row
  features and events. Same code as the firmware, so tuned numbers are firmware numbers.
- `tools/analysis` (Python): plots every recording (forces, states, marks, features, events),
  builds labelled episodes from the marks (slip = P/T/S mark to E or +3 s; tolerance ±0.5 s;
  O marks and quiet MONITOR time are negatives), grid-searches the parameters through the
  replay tool, and reports detection rate, latency, false alarms per minute, per object and
  **leave-one-object-out**, then writes the chosen parameters as a config patch.
- One command runs the whole pipeline on synthetic recordings (plumbing self-test). Synthetic
  results are never reported as tuned thresholds.

## 8. Calls made by the designer
All default thresholds above; settle 500 ms; holdoff 1.5 s; fusion default "either" until tuned;
tighten gain 1.25 and cap 0.9 × Max force; failed tighten → MONITOR; ToF primary feature = raw
frame zone change (the fingers do not move in MONITOR), smoothed measurement secondary.
