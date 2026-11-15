# Phase 2 — Power-up Homing and Calibration Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After any power-up the gripper recovers its multi-turn position with one button (close-to-touch), a one-time calibration sweep records what the SEN0628s see along the whole travel, and that recording gives a position→fingertip-opening table plus a power-up estimate and a running position check.

**Architecture:** Pure, host-tested cores do the maths: `tof_geometry` (Release 3.0.3 sensor geometry ported from JS), `homing_core` (turn resolution), `sweep_core` (finger zones, opening table, position estimate/check). The servo layer applies a RAM `posOffset` so everything above it works in one logical frame. The control task runs Home and Sweep as operator-started procedures (like jog, IDLE/FAULT only); the FSM gets a `homed` input and blocker. The sweep is stored in LittleFS.

**Tech Stack:** Arduino-ESP32 3.x (LittleFS, Preferences), existing SenseGrip modules, host tests `make -C tests`, vanilla JS dashboard.

**Spec:** `docs/superpowers/specs/2026-11-15-sensing-homing-perception-design.md` §4, §6.

## Global Constraints

- Logical position = servo position + `posOffset` (int32, RAM only; unknown after every ESP32 boot → NOT homed). All saved positions are logical. Goals sent to the servo = logical − `posOffset`.
- Not homed: Open and Arm are refused; jog, Home and the first-time sweep (no touch reference yet) are allowed. Blocker text: `not homed — press Home (gripper must be empty)`.
- Distal (fingertip) FSR indices: 5, 3, 4 (`FINGER_FSR_MAP[f][0]`). Proximal: 0, 2, 1.
- Home: speed **5**, current **120**; stop on **any distal FSR ≥ contactForceN for 3 consecutive samples**; a stall (< 10 counts in 300 ms while commanded) **fails** Home; **unstick**: no motion within 300 ms of the first step → one retry at current **415** for at most **500 ms**; timeout **60 s**; resolve with turn = 4096 counts, accept |residual| ≤ **600**; if a SEN0628 estimate exists and points to a different turn → fail, nothing moves; on success open to OPEN zero.
- No command may target beyond OPEN zero (jog excepted when "Ignore limits" is ticked).
- Sweep: from OPEN zero, close at speed **5**, sample every **100** counts (wait for the servo to stop and for 2 new frames per online sensor), end at distal contact (same rule as Home) → touch reference `T`; close limit = `T` − 100 counts toward open; stored in LittleFS `/sweep.bin`.
- Geometry constants (Release 3.0.3): radius 101 mm, z 0, pitch 30°, sensor yaw 90/210/330°, zone step 7.5°, fingertip radius when open 170.28 mm; per-sensor calibration from NVS `senCal2` (`ver`=1; keys `o<i>` orientation, `s<i>` rangeScale, `b<i>` rangeOffset, `y<i>`/`p<i>`/`r<i>` deg, `v<i>` valid; defaults orientation 4, scale 1, offset 0, angles 0).
- Fingers invisible to the SEN0628 within **1000** counts of OPEN zero.
- Position check: only while homed, IDLE or MONITOR, position > OPEN zero + 1000 (closing side), no object present; mismatch for > 1 s → not homed + blocker `position check failed — press Home`.
- Config: new fields appended after `fsrKalmanR`: `int32_t touchRef; bool touchValid;` Config version 6 (v5 blob = prefix up to `touchRef`; same padded-blob handling as v3/v4).
- Pure files must not include Arduino headers. Files: `tof_geometry.cpp` (Makefile gains a `*_geometry.cpp` glob), `homing_core.cpp`, `sweep_core.cpp` (matched by `*_core.cpp`).

## Review Focus

1. **Home with an object or a hand between the fingers** → contact is not at the touch point → residual > 600 → Home fails with the residual in the message, nothing opens. Test `home_rejects_large_residual` (Task 2).
2. **Home pressed with the fingertips already touching** → stops at once, resolves correctly. Test `home_resolves_when_already_at_touch` (Task 2).
3. **Sweep aborted (E-stop, timeout, lost servo)** → no partial sweep replaces the previous one. Test `sweep_rejects_non_monotonic_or_short` (Task 3).
4. **Power-up estimate with the fingers in the first 1000 counts (invisible)** → "no estimate", Home is not cancelled by it. Test `estimate_invalid_without_finger_zones` (Task 3).
5. **Position check while an object hides finger zones** → check is suspended, no false "Home again". Test `check_suspended_when_object_present` (Task 3) + FSM gating in Task 5.

---

### Task 1: SEN0628 geometry core

**Files:** Create `SenseGrip/tof_geometry.h/.cpp`; Test `tests/test_tof_geometry.cpp`; Modify `tests/Makefile` (add `*_geometry.cpp` glob).

**Interfaces — Produces:**
```cpp
struct Vec3 { float x, y, z; };
struct SensorCalib { uint8_t orientation; float rangeScale, rangeOffset, yawDeg, pitchDeg, rollDeg; bool valid; };
SensorCalib defaultCalib();                                   // {4, 1, 0, 0, 0, 0, false}
uint8_t zoneSourceIndex(uint8_t orientation, uint8_t row, uint8_t col);   // = JS matrixSourceIndex
bool    zonePoint(uint8_t sensor, uint8_t row, uint8_t col, float mm, const SensorCalib& c, Vec3& out); // false if mm < 1 after calib
float   horizontalRadius(const Vec3& p);                       // sqrt(x²+y²)
```
Port verbatim from Release 3.0.3 (`git show 17e87bc:SENSE_GRIP_Release_3_0_3_Simple_FSM.ino`, JS lines ~1241–1264): `mechanicalAxes`, `localRay`, `rotateLocal`, `localToWorld`, `makePoseMatrix`, `pointFor`, `matrixSourceIndex`. `zonePoint(s,row,col,mm,c)` = origin(s) + localToWorld(s, localRay(row,col)·d) with d = mm·rangeScale + rangeOffset.

- [ ] **Step 1: Failing tests:** `geom_source_index_orientations` (orientation 0: (2,5)→21; 4 mirror columns: (2,5)→18; 2 rotate 180: (0,0)→63); `geom_sensor_origin` (zonePoint at mm→0 not allowed; for sensor 0 a 100 mm centre ray point (row 3.5 equivalent: average of zones (3,3),(3,4),(4,3),(4,4)) lies inward: horizontalRadius < 101 and z > 0 since forward = −cos30·er + sin30·up); `geom_symmetry` (same zone and mm on sensors 0,1,2 give points with equal horizontalRadius within 1e-3 and z within 1e-3); `geom_range_calibration` (rangeScale 1.1, offset −5: point distance from origin = 100·1.1 − 5 within 1e-3); `geom_rotation_yaw` (yawDeg 5 moves the centre-ray point sideways: its distance from the uncorrected point > 0).
- [ ] **Step 2:** `make -C tests` — FAIL (missing header). **Step 3:** Implement. **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat: SEN0628 geometry core (ported from 3.0.3)`.

### Task 2: Homing core

**Files:** Create `SenseGrip/homing_core.h/.cpp`; Test `tests/test_homing.cpp`.

**Interfaces — Produces:**
```cpp
constexpr int32_t COUNTS_PER_TURN = 4096;
struct HomeResult { bool ok; int32_t posOffset; int32_t residual; const char* why; };
// rawAtTouch: servo position where the fingertips touched; touchRef: saved logical touch position.
HomeResult homeResolve(int32_t rawAtTouch, int32_t touchRef, int32_t maxResidual);
// Turn index of a logical position relative to touchRef (for comparing with the SEN0628 estimate).
int32_t turnOf(int32_t logicalPos, int32_t touchRef);
// Stall rule: true when |pos - posAtWindowStart| < 10 and the window is >= 300 ms old.
bool stalled(int32_t pos, int32_t windowStartPos, uint32_t windowAgeMs);
```
`homeResolve`: posOffset = k·4096 with k = round((touchRef − rawAtTouch)/4096); residual = rawAtTouch + posOffset − touchRef; ok iff |residual| ≤ maxResidual (why = "residual N counts > limit").

- [ ] **Step 1: Failing tests:** `home_resolves_exact_turns` (touchRef 19202, raw 19202 − 2·4096 → offset 8192, residual 0, ok); `home_resolves_with_small_residual` (raw = 19202 − 4096 + 150 → offset 4096, residual 150, ok); `home_rejects_large_residual` (raw = 19202 + 1500 − 4096 → |residual| 1500 → !ok, why non-null); `home_resolves_when_already_at_touch` (raw == touchRef → offset 0, ok); `home_negative_positions` (touchRef −3000, raw = −3000 + 3·4096 → offset −12288); `turn_of` (turnOf(touchRef − 4096·2 − 10, touchRef) == −3 using floor division, turnOf(touchRef, touchRef) == 0); `stall_rule` (Δ 9 at 300 ms → true, Δ 9 at 299 → false, Δ 10 at 400 → false).
- [ ] **Step 2:** FAIL. **Step 3:** Implement. **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat: homing core (turn resolution, stall rule)`.

### Task 3: Sweep core

**Files:** Create `SenseGrip/sweep_core.h/.cpp`; Test `tests/test_sweep.cpp`.

**Interfaces — Consumes:** Task 1 (`zonePoint`, `horizontalRadius`, `SensorCalib`, `zoneSourceIndex`).
**Produces:**
```cpp
constexpr int SWEEP_MAX = 260;                       // ~20 000 counts / 100 + margin
struct SweepSample { int32_t pos; uint16_t mm[3][64]; uint8_t valid;   /* bit s: sensor s frame present */ };
struct SweepModel {
  int n; int32_t openZero, touchRef;
  uint64_t fingerZones[3];                            // bit z set: zone z shows a finger
  float opening[SWEEP_MAX];                           // fingertip radius (mm) at sample i
};
bool  sweepValid(const SweepSample* s, int n, int32_t openZero, bool closePositive);   // n >= 20, positions strictly monotonic toward closing, first within 150 of openZero
void  sweepAnalyze(const SweepSample* s, int n, const SensorCalib cal[3], int32_t openZero, int32_t touchRef, bool closePositive, SweepModel& out);
float openingAt(const SweepModel& m, const SweepSample* s, int32_t pos);            // linear interpolation; below the visible region extrapolate linearly to 170.28 at openZero
struct PosEstimate { bool valid; int32_t pos; float score; };
PosEstimate estimatePosition(const SweepModel& m, const SweepSample* s, const uint16_t frame[3][64], const uint8_t frameValid);
bool  positionMismatch(const SweepModel& m, const SweepSample* s, int32_t pos, const uint16_t frame[3][64], uint8_t frameValid, float thresholdMm);
```
Rules (builder's choice where not stated, record in a comment):
- A zone is a **finger zone** if, over samples more than 1000 counts from OPEN zero with a valid reading (0 < mm < 4000), its reading changes by > 30 mm across the sweep and correlates with position (|Pearson r| ≥ 0.8).
- **Opening** at a sample = minimum `horizontalRadius` of that sample's finger-zone points (fingertips are the innermost finger parts); samples with no finger point are filled by interpolation, and the first 1000 counts by linear extrapolation to 170.28 mm at OPEN zero.
- **Estimate:** score of a sample = mean |frame − sample| over finger zones valid in both; best = lowest score; valid only if ≥ 6 finger zones were compared and the best sample is > 1000 counts from OPEN zero.
- **Mismatch:** compare the live frame with the sample nearest `pos` over finger zones; mismatch if the mean |diff| > `thresholdMm` (caller uses 25 mm) and ≥ 6 zones compared; with < 6 zones compared it returns false (suspended).

- [ ] **Step 1: Failing tests** with a synthetic sweep built in the test: 200 samples, pos = 2000 + 100·i; zones 10,11,18,19 of every sensor read `400 − 1.5·i` (finger), all other zones read 1200 constant (background), zones invalid (0) for pos < 3000 to model invisibility.
  `sweep_finger_zones_found` (exactly zones 10,11,18,19 flagged per sensor); `sweep_opening_monotonic` (opening non-increasing with position; opening at openZero = 170.28 ± 0.01); `sweep_estimate_recovers_position` (frame = sample 120 + 3 mm noise pattern → estimate within ±100 counts of sample 120's pos); `estimate_invalid_without_finger_zones` (frame with all finger zones 0 → valid == false); `check_detects_mismatch` (frame of sample 150 checked at pos of sample 80 → true); `check_suspended_when_object_present` (frame with finger zones replaced by 0/occluded → false); `sweep_rejects_non_monotonic_or_short` (19 samples → false; a backwards step → false; first sample 400 counts from openZero → false).
- [ ] **Step 2:** FAIL. **Step 3:** Implement. **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat: sweep core (finger zones, opening table, position estimate/check)`.

### Task 4: Logical position frame in the servo layer + config fields

**Files:** Modify `SenseGrip/servo.h/.cpp`, `SenseGrip/config_core.h/.cpp`, `SenseGrip/config.cpp`, `SenseGrip/control.cpp`; Test `tests/test_config.cpp`.

**Interfaces — Produces:**
- `void servoSetOffset(int32_t posOffset); int32_t servoGetOffset();` — `ServoSnapshot.fb.position` is **logical**; new `int32_t rawPosition` in `ServoSnapshot`; `servoMoveTo`/`servoJogTo`/`servoRunAt` take logical goals and subtract the offset; offset starts 0.
- `GripConfig`: `int32_t touchRef; bool touchValid;` appended (defaults 0/false); `GRIP_CONFIG_V5_SIZE = offsetof(touchRef)`, `GRIP_CONFIG_V5_BLOB_SIZE` padded to `alignof(GripConfig)`; `CFG_VERSION 6`; v5 keeps limits.
- `control.cpp`: on `servoSetOffset` reset the jump-guard baseline (`havePos = false`).
- [ ] **Step 1: Failing tests:** `config_touch_defaults` (touchRef 0, touchValid false); `config_v5_blob_sizes` (`GRIP_CONFIG_V5_SIZE == offsetof(GripConfig, touchRef)`, `GRIP_CONFIG_V5_BLOB_SIZE % alignof(GripConfig) == 0`, `GRIP_CONFIG_V5_BLOB_SIZE >= GRIP_CONFIG_V5_SIZE`).
- [ ] **Step 2:** FAIL. **Step 3:** Implement (loader: v6 full, v5 → V5 sizes, v3/v4 → V4 sizes, v1/v2 → V2). **Step 4:** PASS; push, CI compiles.
- [ ] **Step 5:** Commit `feat: logical servo position frame and touch reference config (v6)`.

### Task 5: FSM homed input, Home procedure

**Files:** Modify `SenseGrip/fsm_core.h/.cpp`, `SenseGrip/control.h/.cpp`, `SenseGrip/web.cpp`; Test `tests/test_fsm.cpp`.

**Interfaces:**
- Consumes: Task 2 (`homeResolve`, `stalled`, `turnOf`), Task 4 (`servoSetOffset`, `rawPosition`, `touchRef/touchValid`).
- Produces: `FsmInputs.homed` (bool); blocker `BLK_NOT_HOMED` text `not homed — press Home (gripper must be empty)` listed first after servo blockers; OPEN refused when !homed (reason `Open refused: not homed`). `ControlRequest::HOME`; `ControlStatus` gains `bool homed; int32_t homeResidual; char homeMsg[64]; bool homing;`; route `POST /api/servo/home`; `/api/status` `home:{homed,homing,residual,msg}`.
- Home runs inside the control loop as a small procedure (not an FSM state): IDLE/FAULT only, disarmed, servo online + mode 0, `touchValid`, no FSR reading contact at start. Steps per Global Constraints; on success `servoSetOffset(result.posOffset)`, `homed = true`, then queue `Cmd::OPEN`. Any request (E-stop, jog, FSM command) aborts Home with torque off and message.
- Jump guard: a detected jump sets `homed = false`.

- [ ] **Step 1: Failing FSM tests:** `fsm_not_homed_blocks_and_refuses_open` (homed=false → blocker present, OPEN → NONE with reason containing "not homed"); `fsm_homed_no_blocker` (homed=true → no such blocker). Update the `ready()` helper to `homed = true`.
- [ ] **Step 2:** FAIL. **Step 3:** Implement FSM part, then the control procedure and web route. **Step 4:** host PASS; push, CI compiles.
- [ ] **Step 5:** Commit `feat: Home (close-to-touch) and not-homed blocker`.

### Task 6: Calibration sweep run, storage, power-up estimate and running check

**Files:** Create `SenseGrip/sweep_store.h/.cpp` (LittleFS + `senCal2` loader); Modify `SenseGrip/control.cpp`, `SenseGrip/control.h`, `SenseGrip/web.cpp`, `SenseGrip/SenseGrip.ino` (LittleFS.begin(true) in setup).

**Interfaces:**
- Consumes: Task 1, Task 3, Task 5 (Home procedure pattern, `homed`).
- Produces:
  - `bool sweepStoreSave(const SweepSample* s, int n, const SweepModel& m); bool sweepStoreLoad(SweepSample* s, int& n, SweepModel& m); bool tofCalibLoad(SensorCalib out[3]);` — samples buffer allocated once in PSRAM (`ps_malloc`, SWEEP_MAX samples ≈ 100 KB).
  - `ControlRequest::SWEEP` (route `POST /api/servo/sweep`): allowed when IDLE/FAULT, disarmed, OPEN zero set and (homed or not `touchValid` = first-time setup with offset 0). Moves to OPEN zero first, then steps per Global Constraints, records each sample from `tofGet()` (wait for `ok` counters to advance by 2 per online sensor, max 1 s), stops on distal contact. Validates with `sweepValid`; only then saves, sets `touchRef`/`touchValid`, close limit = touchRef − 100 toward open, `limitsValid`, `homed = true`, analyzes, opens to OPEN zero. Aborts (no save) on any request, timeout 180 s, servo offline.
  - Power-up estimate: once at boot when the sweep loads and every 500 ms until homed: `estimatePosition` from the latest frames → `ControlStatus.estimate:{valid,pos,turn}`; Home fails with `SEN0628 disagrees: turn a vs b` when valid and `turnOf(estimate.pos, touchRef) != turnOf(rawAtTouch+posOffset, touchRef)`.
  - Running check every 200 ms per Global Constraints with `positionMismatch(…, 25.0f)`; > 1 s continuous → `homed = false`, message `position check failed — press Home`.
  - `GET /api/sweep` → `{n, openZero, touchRef, positions[], opening[], fingerZones:["hex16"×3]}` (no raw frames).
- [ ] **Step 1:** Implement. **Step 2:** host suite still PASS; push; CI compiles.
- [ ] **Step 3:** Commit `feat: calibration sweep, SEN0628 power-up estimate and running position check`.

### Task 7: Dashboard

**Files:** Modify `SenseGrip/web_pages.h`.

- Combined + Servo tabs: **Home** button (disabled with first-blocker tooltip when not allowed), homed badge, last residual and message, SEN0628 estimate ("estimate: turn n, pos p" or "no estimate").
- Servo tab: **Run calibration sweep** button (confirm: "gripper empty, nothing in view"), progress (sample count, position) while running; **opening vs position** plot from `/api/sweep` (canvas, mm vs counts, touch and OPEN marked) — thesis figure.
- SEN0628 tab: finger zones outlined on each heatmap (from `fingerZones`).
- [ ] **Step 1:** Implement. **Step 2:** `node --check`; no `)HTML"`. **Step 3:** mock smoke: Home posts `/api/servo/home`, sweep plot draws from a mock `/api/sweep`, finger-zone outline visible, no console errors, 390 px no horizontal scroll. **Step 4:** Commit `feat: Home, sweep plot and finger zones on the dashboard`.

### Task 8: Docs and hardware checklist

**Files:** `README.md`, `docs/HARDWARE_CHECKLIST.md`.
- README: power-up flow (Home), first-time setup (jog to open → Set OPEN zero → Run calibration sweep), what the sweep records, estimate and running check, new blockers and routes, LittleFS file.
- Checklist: first-time sweep (gripper empty, note touch position and the plot shape); Home after 3 servo power-cycles from different positions (note residuals); Home with an object in the gripper must fail; ESP32 reset without servo power loss → Home again; jump guard: with the ESP32 on, power-cycle only the servo → "not homed" appears; running check: hold an object in view while idle → no "position check failed".
- [ ] **Step 1:** Write. **Step 2:** `make -C tests` PASS. **Step 3:** Commit `docs: homing and calibration sweep`; push.
