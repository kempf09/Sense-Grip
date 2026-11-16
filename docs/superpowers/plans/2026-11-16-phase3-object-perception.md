# Phase 3 — Object Perception and Object-Aware Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** See the object with the three SEN0628, measure width / size class / coarse shape, move the fingertips to a stand-off from it at full speed, close slowly to contact and more slowly to the class's target force; show class, shape and contact pattern on the dashboard.

**Architecture:** New pure, host-tested modules do the maths (`object_core`: object points → width/class/shape; `contact_core`: wrap/pinch pattern; `sweep_core` gains opening → position inversion and per-position background). The FSM consumes a ready-made `GraspPlan` in `FsmInputs` (built by `control.cpp` from the measurement, the class profile, the operator's RAM-only override/delicate switches) and latches it when it leaves START. Web/UI only display and set.

**Tech Stack:** Arduino-ESP32 3.x (C++17), FreeRTOS tasks, LittleFS, Preferences/NVS; host tests with g++ (`make -C tests`, `tests/check.h`); CI `.github/workflows/build.yml` compiles esp32s3 (huge_app) and runs host tests.

**Spec:** `docs/superpowers/specs/2026-11-15-sensing-homing-perception-design.md` §7 (and §4 for geometry/hardware facts). Read both.

## Global Constraints

- Branch `feature/continuous-approach-touch-window`; commit after every task with the trailer lines
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_018Nxc3BNRHZQ3pvMujtS6sE`. No PR.
- Servo: position mode 0 only; never write the mode during motion. Speed unit 0.732 RPM, max 61 (`SPEED_MAX`). Current unit 6.5 mA, stall 2.7 A ≈ 415.
- Pure modules (`*_core.cpp`, `*_geometry.cpp`, ...) must not include Arduino headers; the tests Makefile globs `*_core.cpp`.
- SEN0628 range error ±10 % of the reading: background margin max(15 mm, 20 % of background); class boundaries keep the larger class within 10 % of a boundary; pre-touch stand-off = x + 10 % of the object range.
- Only points within 100 mm of the gripper axis and above the palm (z > 5 mm in the `tof_geometry` world frame; sensors at z = 0, rays point up) are object points.
- Class defaults: small < 50 mm, medium 50–90 mm, large > 90 mm. Profiles (stand-off mm / target N / max current / grasp speed): small 10 / 1.0 / 200 / 5, medium 10 / 2.0 / 260 / 5, large 10 / 4.0 / 350 / 5, delicate 15 / 0.4 / 150 / 3. Slow-to-contact speed 15. Approach speed 61 (existing `approachSpeed`).
- Override (auto/small/medium/large) and Delicate are RAM only (lost on reboot), shown as badges on Combined; settings live on the SEN0628 tab.
- Every refusal names its reason (existing `say()` / FSM reason / blocker system). Never pick a force from size alone when the measurement is missing: no measurement and no override → START aborts.
- Existing safety unchanged: force limit on max(raw, filtered), travel limits, timeouts, E-stop.

## Review Focus

1. **No sweep / no measurement:** START must abort with "object not measured (no sweep / no points)" unless a class override is set; with an override and no measurement the approach step is skipped (slow close from where the jaw is). Test in Task 5.
2. **Object bigger than the opening at the current position** (stand-off opening ≥ current opening, e.g. drill at OPEN): skip the fast approach, go straight to the slow close; never move toward open. Test in Task 3 (`positionForOpening` clamps) and Task 5.
3. **Class flickers at a boundary** during START: START waits for 300 ms of one class and gives up after 3 s with "object size not stable"; disarms. Test in Task 5.
4. **Class target force ≥ maxForceN** (default 3 N vs large 4 N): config refuses the save with a named reason, and default `maxForceN` becomes 8 N. Test in Task 1.
5. **One SEN0628 offline in START:** measurement uses the remaining sensors (width from those that see the object); shape is "unknown" with fewer than two sensors. Test in Task 2.

---

### Task 1: Config v7 — class thresholds, class profiles, contact speed

**Files:**
- Modify: `SenseGrip/config_core.h`, `SenseGrip/config_core.cpp`, `SenseGrip/config.cpp`, `SenseGrip/web.cpp` (config field table `F_`)
- Test: `tests/test_config.cpp`

**Interfaces:**
- Produces (appended to `GripConfig` after `touchValid`, in this order):
  `uint16_t smallMaxMm, mediumMaxMm; uint16_t contactSpeed;`
  then for each of `small`, `medium`, `large`, `delicate`: `float <p>StandoffMm, <p>ForceN; uint16_t <p>MaxCurrent, <p>GraspSpeed;`
- `enum class ObjClass : uint8_t { SMALL = 0, MEDIUM = 1, LARGE = 2, DELICATE = 3 };` and
  `struct ClassProfile { float standoffMm, forceN; uint16_t maxCurrent, graspSpeed; };`
  `ClassProfile profileOf(const GripConfig&, ObjClass);` in `config_core.h`.
- `const char* profileProblem(const GripConfig&);` nullptr or a reason.
- Constants `GRIP_CONFIG_V6_SIZE = offsetof(GripConfig, smallMaxMm)` and `GRIP_CONFIG_V6_BLOB_SIZE` (V6 size rounded up to `alignof(GripConfig)`), same pattern as V5.

- [ ] **Step 1: Write failing tests** in `tests/test_config.cpp`:
  - `config_v7_defaults`: `smallMaxMm==50`, `mediumMaxMm==90`, `contactSpeed==15`, `maxForceN==8.0f`; `profileOf(c, ObjClass::LARGE)` = {10, 4.0, 350, 5}; `DELICATE` = {15, 0.4, 150, 3}; `!clampConfig(c)`; `profileProblem(c)==nullptr`.
  - `config_profile_problem`: `largeForceN = 9` with `maxForceN = 8` → non-null; `delicateForceN = 0.2` with `contactForceN = 0.25` → non-null; `smallMaxMm = 95, mediumMaxMm = 90` → non-null.
  - `config_v6_size_constants`: `GRIP_CONFIG_V6_SIZE == offsetof(GripConfig, smallMaxMm)`, `GRIP_CONFIG_V6_BLOB_SIZE >= GRIP_CONFIG_V6_SIZE`, `GRIP_CONFIG_V6_BLOB_SIZE % alignof(GripConfig) == 0`.
- [ ] **Step 2:** `make -C tests` → FAIL (missing fields).
- [ ] **Step 3: Implement.** Defaults per Global Constraints; `maxForceN` default 3.0 → 8.0. Clamp ranges: thresholds 20..250; standoff 0..50 mm; force 0.1..30 N; maxCurrent 50..415; graspSpeed and contactSpeed 1..`SPEED_MAX`. `profileProblem`: smallMaxMm < mediumMaxMm; each profile `forceN > contactForceN` and `forceN < maxForceN` (message names the class, e.g. "large target force must be below Max force N"). `config.cpp`: `CFG_VERSION 7`; v6 blobs load with `stored = GRIP_CONFIG_V6_BLOB_SIZE`, `keep = GRIP_CONFIG_V6_SIZE` (new fields keep defaults). Web `CONFIG` handling in `control.cpp` refuses a config whose `profileProblem` is non-null (`say(c,false,"config refused: %s",...)`). Add the 19 new fields to the `F_` table in `web.cpp` (types `T_U16`/`T_F`).
- [ ] **Step 4:** `make -C tests` → all pass.
- [ ] **Step 5: Commit** `feat: config v7 class thresholds and grasp profiles`.

### Task 2: `object_core` — object points, width, class, shape

**Files:**
- Create: `SenseGrip/object_core.h`, `SenseGrip/object_core.cpp`
- Test: `tests/test_object.cpp`

**Interfaces:**
- Consumes: `zonePoint`, `horizontalRadius`, `zoneSourceIndex`, `SensorCalib`, `Vec3` (`tof_geometry.h`); `ObjClass` (Task 1).
- Produces:
  ```cpp
  enum class Shape : uint8_t { UNKNOWN, ROUND, ELONGATED, IRREGULAR };
  struct ObjectMeasure {
    bool valid;             // >= 3 object points in total
    float widthMm;          // 2 x mean over sensors of the per-sensor median point radius
    float rangeMm;          // median sensor range (mm) of all object points
    float sensorRadiusMm[3];// NAN where the sensor has < 2 points
    float heightMm;         // max z - min z of all object points
    uint8_t nPoints; uint64_t zones[3];   // raw zone bits that are object points
    Shape shape;
  };
  // background: per-sensor per-zone empty-gripper reading at this position (0 = none);
  // fingerZones: excluded; frameValid bit s = sensor s frame present.
  ObjectMeasure measureObject(const uint16_t frame[3][64], uint8_t frameValid,
                              const uint16_t background[3][64], const uint64_t fingerZones[3],
                              const SensorCalib cal[3]);
  ObjClass classify(float widthMm, uint16_t smallMaxMm, uint16_t mediumMaxMm);  // never DELICATE
  const char* shapeName(Shape); const char* className(ObjClass);
  ```
- Rules (spec §7.1/7.2): zone is an object point when not a finger zone, reading in 1..3999, and (background 0 or reading < background − max(15, 0.2·background)); the 3-D point (raw zone → display row/col via `zoneSourceIndex` and `cal[s].orientation`, as `sweep_core`'s `rawToDisplay`) must have `horizontalRadius ≤ 100` and `z > 5`. Classify with the 10 % band: `w >= 0.9*mediumMax` → LARGE, else `w >= 0.9*smallMax` → MEDIUM, else SMALL. Shape: < 2 sensors with radius → UNKNOWN; `heightMm >= 1.8*widthMm` → ELONGATED; max/min of the sensor radii ≤ 1.25 → ROUND; else IRREGULAR.

- [ ] **Step 1: Write failing tests** in `tests/test_object.cpp`. Test helper `castCylinder(frame, R, z0, z1)`: for every sensor and zone, step `mm` from 1 to 3999 in 0.5 mm steps and write the first `mm` whose `zonePoint` has `horizontalRadius <= R` and `z0 <= z <= z1` (else 0); background frame all 1200 (`castBackground`). Tests:
  - `object_cylinder_width`: R = 30, z 10..60 → `valid`, `|widthMm − 60| ≤ 10`, `classify(widthMm,50,90) == MEDIUM` (60 ≥ 45).
  - `object_small_and_large`: R = 15 → SMALL; R = 50 → LARGE.
  - `object_class_band`: `classify(46,50,90)==MEDIUM`, `classify(44,50,90)==SMALL`, `classify(82,50,90)==LARGE`.
  - `object_background_margin`: frame = background − 10 mm everywhere → `!valid`.
  - `object_elongated`: R = 15, z 10..200 → `shape == ELONGATED` (only if `heightMm ≥ 1.8·width`; assert the heightMm/width ratio first).
  - `object_one_sensor_offline`: R = 30 with `frameValid = 0b011` → `valid`, `isnan(sensorRadiusMm[2])`, width still within ±10 of 60; with `0b001` → `shape == UNKNOWN`.
  - `object_ignores_finger_zones`: mark every zone of sensor 0 as finger zone → `zones[0] == 0`.
- [ ] **Step 2:** `make -C tests` → FAIL (no `object_core.h`).
- [ ] **Step 3: Implement** `object_core.cpp` per the rules above (median via a small local array + `std::nth_element`).
- [ ] **Step 4:** `make -C tests` → pass. If the cylinder widths miss ±10 mm, check the raw↔display mapping before widening the tolerance.
- [ ] **Step 5: Commit** `feat: object measurement (width, class, shape) from SEN0628 frames`.

### Task 3: `sweep_core` — opening → position, background at a position

**Files:**
- Modify: `SenseGrip/sweep_core.h`, `SenseGrip/sweep_core.cpp`
- Test: `tests/test_sweep.cpp`

**Interfaces:**
- Produces:
  ```cpp
  // Logical position where the fingertip opening first falls to openingMm, moving from `from`
  // toward closing. Returns `from` when the opening at `from` is already <= openingMm (never
  // moves toward open); returns the last sample's position when the opening never gets that small.
  int32_t positionForOpening(const SweepModel& m, const SweepSample* s, int32_t from, float openingMm);
  // Empty-gripper reading per zone at pos: the nearest sample's frame plus delta (clamped to
  // 0..3999; 0 stays 0, sensors absent in that sample give 0).
  void backgroundAt(const SweepModel& m, const SweepSample* s, int32_t pos,
                    const int16_t delta[3][64], uint16_t out[3][64]);
  ```
- [ ] **Step 1: Failing tests** (reuse `makeSweep`/`analyze`): `opening_to_position_inverts` — for i = 60, `p = positionForOpening(m, v, 2000, m.opening[60])`, `|openingAt(m,v,p) − m.opening[60]| ≤ 1` and p within 100 of `v[60].pos`; `opening_to_position_never_opens` — from `v[100].pos` with `m.opening[20]` returns `v[100].pos`; `opening_to_position_clamps_closed` — opening −1 → `v[N-1].pos`; `background_at_adds_delta` — delta +5 on zone 30 of sensor 1 → `out[1][30] == 1205`, zone 10 at sample with 0 stays 0.
- [ ] **Step 2:** `make -C tests` → FAIL.
- [ ] **Step 3: Implement** (walk samples from the one nearest `from` toward closing; interpolate between the bracketing samples).
- [ ] **Step 4:** pass.
- [ ] **Step 5: Commit** `feat: sweep opening inversion and per-position background`.

### Task 4: `contact_core` — wrap / pinch pattern

**Files:**
- Create: `SenseGrip/contact_core.h`, `SenseGrip/contact_core.cpp`
- Test: `tests/test_contact.cpp`

**Interfaces:**
- Produces:
  ```cpp
  enum class FirstContact : uint8_t { NONE, PROXIMAL, DISTAL, BOTH };   // per finger, latched
  enum class Pattern : uint8_t { NONE, WRAP, PINCH, MIXED };
  // Latches the first contact kind per finger (unchanged once != NONE).
  void updateFirstContact(FirstContact f[3], const bool proximal[3], const bool distal[3]);
  Pattern patternOf(const FirstContact f[3]);   // any PROXIMAL -> WRAP; all touched DISTAL -> PINCH; NONE if none
  // Expected: LARGE -> WRAP, SMALL/DELICATE -> PINCH, MEDIUM -> either. NONE pattern -> true.
  bool patternConsistent(Pattern p, ObjClass c);
  const char* patternName(Pattern);
  ```
- [ ] **Step 1: Failing tests:** latching (distal then both stays DISTAL); `{PROXIMAL, DISTAL, NONE}` → WRAP; `{DISTAL, NONE, DISTAL}` → PINCH; `{BOTH, DISTAL, DISTAL}` → MIXED; consistency table incl. `!patternConsistent(PINCH, LARGE)`.
- [ ] **Step 2–4:** FAIL → implement → pass.
- [ ] **Step 5: Commit** `feat: contact pattern (wrap/pinch)`.

### Task 5: FSM — GraspPlan, START measurement, two-step PRE_TOUCH, class force

**Files:**
- Modify: `SenseGrip/fsm_core.h`, `SenseGrip/fsm_core.cpp`
- Test: `tests/test_fsm.cpp`

**Interfaces:**
- Consumes: `ObjClass`, `ClassProfile` (Task 1).
- Produces:
  ```cpp
  struct GraspPlan {
    bool valid;          // usable for a grasp (measured, or override set)
    ObjClass cls;        // effective class (override/delicate applied)
    bool approach;       // a fast approach goal exists
    int32_t approachGoal;// logical position (from positionForOpening)
    ClassProfile prof;
  };
  // FsmInputs += GraspPlan plan; const char* planWhy (reason when !plan.valid).
  enum class PreTouchStep : uint8_t { APPROACH, SLOW };
  PreTouchStep Fsm::preTouchStep() const; const GraspPlan& Fsm::plan() const;  // latched plan
  ```
- Behaviour:
  - START: if `!in.plan.valid` → keep waiting; track `classSinceMs_` reset whenever `in.plan.cls` changes or plan invalid; leave START when the plan has been valid with the same class for ≥ 300 ms (`START_STABLE_MS`), latching `plan_`. After 3000 ms (`START_TIMEOUT_MS`) without that: disarm, IDLE, reason `"Start aborted: <planWhy>"` if the plan was never valid, else `"Start aborted: object size not stable"`. Blockers and disarm checks stay first.
  - PRE_TOUCH entry: if `plan_.approach` and the goal is on the closing side of the present position by more than `openToleranceCounts` → step APPROACH, `MOVE` to `approachGoal` at `approachSpeed`, `plan_.prof.maxCurrent`; else step SLOW, `RUN` at `contactSpeed`, `plan_.prof.maxCurrent`. In APPROACH, `|position − goal| ≤ openToleranceCounts` → step SLOW with `RUN contactSpeed` (reason "pre-touch position reached: slow to contact"). Contact in either step → GRASP. Close-limit and timeout checks unchanged (whole PRE_TOUCH).
  - GRASP: `RUN` at `plan_.prof.graspSpeed`, `plan_.prof.maxCurrent`; `graspReached` uses `plan_.prof.forceN` (not `cfg_.graspForceN`). MONITOR HOLD unchanged.
- [ ] **Step 1: Update helpers and write failing tests.** In `test_fsm.cpp`, `ready()` sets `in.plan = {true, ObjClass::MEDIUM, false, 0, {10, 2.0f, 260, 10}}`; `preTouchFsm(t0)` steps START at t0 and t0+300. New tests:
  - `fsm_start_waits_for_stable_class` — class alternates SMALL/MEDIUM every 100 ms → still START at 2900; at 3000 → IDLE, `!armed()`, reason contains "not stable".
  - `fsm_start_aborts_without_measurement` — `plan.valid=false`, `planWhy="object not measured"` → at 3000 IDLE, reason contains "object not measured".
  - `fsm_pretouch_fast_then_slow` — plan `{approach=true, approachGoal=9000, cls=SMALL, prof={10,1.0,200,10}}`: entry action `MOVE` position 9000 speed 61 torque 200; at position 8995 → action `RUN` speed 15, `preTouchStep()==SLOW`.
  - `fsm_pretouch_goal_behind_skips_approach` — position 9500, goal 9000 → entry action `RUN` speed 15.
  - `fsm_contact_during_approach_goes_grasp` — contact before goal → GRASP, action `RUN` speed `prof.graspSpeed`.
  - `fsm_grasp_uses_class_force` — prof.forceN 1.0, avg 1.2 N, 2 channels, `graspSamples` new samples → MONITOR; with `cfg.graspForceN = 5` unchanged result.
  - `fsm_delicate_grasp_speed` — plan cls DELICATE prof `{15,0.4,150,3}` → GRASP action speed 3, torque 150.
  Adapt existing tests that assumed START → PRE_TOUCH in one step, or `approachSpeed`/`graspSpeed` actions, to the plan values.
- [ ] **Step 2:** `make -C tests` → new tests FAIL.
- [ ] **Step 3: Implement** in `fsm_core.cpp`.
- [ ] **Step 4:** `make -C tests` → all pass.
- [ ] **Step 5: Commit** `feat: FSM grasp plan, stable START, fast approach then slow contact`.

### Task 6: Control glue, background capture, web API

**Files:**
- Modify: `SenseGrip/control.h`, `SenseGrip/control.cpp`, `SenseGrip/sweep_store.h/.cpp`, `SenseGrip/web.cpp`

**Interfaces:**
- Consumes: Tasks 1–5; `sweepModel()`, `sweepSamples()`, `sweepLoaded()`, `tofCalibLoad`/`c.cal`.
- Produces:
  - `sweep_store`: `const int16_t (*bgDelta())[64]` (RAM, zero by default), `bool bgDeltaSave(const int16_t d[3][64])` → LittleFS `/bgdelta.bin` (magic `"BGD1"`), loaded with the sweep; deleted/zeroed by `sweepStoreCommit`.
  - `ControlRequest` kinds `CAPTURE_BACKGROUND`, `SET_OVERRIDE` (field `uint8_t cls`: 0 auto, 1 small, 2 medium, 3 large), `SET_DELICATE` (field `bool on`).
  - `ControlStatus` += `ObjectMeasure obj; uint8_t objClsDetected; uint8_t objClsEffective; bool objForced, delicate; uint8_t pattern; bool patternOk; uint8_t firstContact[3]; uint8_t preTouchStep;`.
- Behaviour:
  - On each new ToF frame set (any `ok` counter changed) and `sweepLoaded() && homed`: `backgroundAt(... sv.fb.position, bgDelta() ...)`, `measureObject(...)`, store in `c.obj`. Not homed / no sweep → `obj.valid=false`, `planWhy` "no sweep / not homed".
  - Build `GraspPlan` each loop: effective class = DELICATE if delicate, else override if set, else `classify(width)`; `valid` = measured or override; `approach` = measured && sweep loaded; `approachGoal = positionForOpening(model, samples, position, width/2 + standoff + 0.1*rangeMm)` clamped to the close side of `closeLimit` (never past it). `planWhy`: "object not measured (no points in view)" / "no sweep stored" / "not homed".
  - `CAPTURE_BACKGROUND`: refused unless IDLE, homed, sweep loaded, no FSR contact; delta = frame − nearest-sample frame for non-finger zones where both are valid (else 0); save; `say` with the number of zones changed.
  - Contact pattern: reset `firstContact` on START entry; update during PRE_TOUCH/GRASP from proximal/distal FSR contact (enabled channels, `contactForceN`).
  - Web: `POST /api/object/override?cls=auto|small|medium|large`, `POST /api/object/delicate?on=0|1`, `POST /api/object/captureBackground`; status JSON adds `"object":{valid,width,range,height,shape,cls,clsDetected,forced,delicate,zones:[hex×3],pattern,patternOk,first:[..],step}` and `fsm.preTouchStep`.
- [ ] **Step 1:** Implement; `make -C tests` still passes (no pure code changed here).
- [ ] **Step 2:** Push and check CI compiles esp32s3 (`build` workflow green).
- [ ] **Step 3: Commit** `feat: object measurement, grasp plan and background capture in the control task`.

### Task 7: Dashboards

**Files:**
- Modify: `SenseGrip/web_pages.h`

- [ ] **Step 1: SEN0628 tab** — panel "Object classification": detected width / range / height / shape / class (live); class thresholds and the four profiles (config fields from Task 1, saved via the existing config POST, errors shown from the reply); Override select (auto/small/medium/large) and Delicate checkbox (RAM-only, label "not kept after restart"); Capture background button (confirm "gripper empty, nothing in view?"). Object zones (`object.zones`) outlined in a distinct colour on the 8×8 heatmaps, next to the finger-zone outlines.
- [ ] **Step 2: Combined tab** — class, width mm, shape; badges "DELICATE" and "CLASS: LARGE (forced)" when set, detected class beside it; PRE_TOUCH step ("approach"/"slow"); contact pattern with per-finger first contact and a ✓/⚠ consistency mark.
- [ ] **Step 3:** Remove `graspForceN` from the FSR config list (class profiles replace it; the field stays in `GripConfig`).
- [ ] **Step 4:** Push; CI green. **Commit** `feat: classification settings and object view on the dashboards`.

### Task 8: Docs and hardware checklist

**Files:**
- Modify: `README.md`, `docs/HARDWARE_CHECKLIST.md`

- [ ] **Step 1:** README section "Object perception and grasp": what the classes mean, stand-off rule (x + 10 % of range), override / delicate (RAM-only), capture background, START refusal reasons, the new default `maxForceN` 8 N (stored configs keep their old value: raise it to above the large-class force).
- [ ] **Step 2:** Checklist steps 24–30: capture background; width of each demo object (egg, tube, mug, apple, drill handle, asymmetric) noted against a ruler; class and shape shown; START refusal with nothing measurable; a grasp per class showing "approach" then "slow" and stopping at the class force; delicate egg grasp; drill with override LARGE; contact pattern wrap vs pinch.
- [ ] **Step 3: Commit** `docs: phase 3 usage and hardware checklist`.
