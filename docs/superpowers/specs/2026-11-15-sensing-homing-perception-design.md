# SENSE-GRIP: FSR filtering, power-up homing, object perception — design

Date: 2026-11-15
Branch: `feature/continuous-approach-touch-window`
Builds on: `2026-11-14-gripper-fsm-rewrite-design.md` (modular firmware, FSM, dashboards)
Status: draft for approval

## 1. Why

The gripper is the subject of the operator's thesis. The deliverable is a **recorded video demo**
(not live) grasping six objects: **egg, tube, mug, apple, drill (by its handle, ~500 g), and an
asymmetric object**. The thesis also needs content that shows the sensing, not only the motion.

Three things are missing today:

1. **FSR signal processing** to present in the thesis (filters, side-by-side comparison).
2. **A position reference after power-up.** Hardware tests showed the CF35-12 keeps its angle
   within one turn but loses the turn count at power-up. Travel is ~20 000 counts (~5 turns), so
   after every power-up the firmware does not know where the fingers are; saved limits are unusable.
3. **Object-aware grasping:** see the object with the three SEN0628, classify it by size (and
   shape), move the fingers to a pre-touch distance from it, then close slowly to contact and
   more slowly to the class's target force.

## 2. Work order (agreed)

| Phase | Content | Depends on |
|---|---|---|
| 1 | FSR filtering + live raw/filtered comparison | — |
| 2 | Power-up homing (close-to-touch) + calibration sweep + SEN0628 position check | — |
| 3 | Object perception (size, shape), pre-touch approach, speed profile, delicate mode, contact pattern | 2 |
| 4 (later, not in this doc) | CSV data logging of a grasp; slip detection | 1–3 |

Each phase ships and is tested on hardware before the next starts.

## 3. Goals, non-goals, anti-goals

**Goals**
- FSR filters selectable at runtime, raw and filtered visible together, the FSM uses the filtered value.
- After any power-up, one button press brings back a correct multi-turn position (±tens of counts).
- A running SEN0628-based check that warns when the position and what the sensors see disagree.
- A position → fingertip-opening table built by one calibration sweep.
- Object width (and a coarse shape class) from the SEN0628, classes small / medium / large.
- Approach to "fingertips x mm from the object surface", then slow to contact, slower to force.

**Non-goals**
- Individual finger positions (one servo drives all three underactuated fingers; only the motor
  position is controlled). Per-finger *contact pattern* is shown, not controlled.
- Live-demo robustness (video recording; retakes are acceptable).
- Slip detection and data logging (phase 4, separate design).

**Anti-goals**
- Moving the motor at power-up without the operator pressing a button.
- Crushing the egg: force must never be chosen from size alone without the delicate override.
- Silent failure: every refused step names its reason (as in the existing blocker system).

## 4. Hardware facts this design relies on

- Servo: position mode 0 only (the firmware never writes the mode during motion; a mode write
  resets the count). Speed unit 0.732 RPM (max 61), accel unit 8.7 °/s² (50 used),
  reg 44 = max current in 6.5 mA units. Stall current 2.7 A (≈ 415 units).
- Servo position after power-up = true angle within the turn, turn count lost.
- The jaw is **not back-drivable**: with torque off the fingers stay where they are.
- **Fully open has no mechanical stop.** **Fully closed: fingertips touch each other**; at least
  one distal (fingertip) FSR shows it. Distal FSR indices (pins.h `FINGER_FSR_MAP[f][0]`):
  finger 1 = idx 5 (GPIO10), finger 2 = idx 3 (GPIO5), finger 3 = idx 4 (GPIO6).
  Proximal (base) = idx 0, 2, 1.
- SEN0628 geometry (from Release 3.0.3 3D page, confirmed correct): sensors at radius 101 mm,
  yaw 90/210/330°, pitched 30°, FoV 60°, 8×8 zones at 7.5° steps; palm radius 50 mm; fingers at
  yaw 30/150/270°, fingertip radius 170.28 mm when open. Per-sensor orientation/range
  calibration from NVS namespace `senCal2` (orientation, rangeScale, rangeOffset, yaw/pitch/roll
  corrections) — reuse it. Reference implementation of the geometry: Release 3.0.3 `.ino`
  (git `17e87bc`), functions `mechanicalAxes`, `localRay`, `localToWorld`, `makePoseMatrix`,
  `pointFor`, `matrixSourceIndex` (lines ~1241–1264).
- The SEN0628 cannot see the fingers in the first ~1 000 counts from open; beyond that the
  fingers are visible.
- Objects are roughly centred, either resting on the palm or held by hand in the sensors' zone.
  Sensors are 101 mm from the centre and can usually still see the object while it is grasped.

## 5. Phase 1 — FSR filtering

**Sampling:** raise FSR sampling from 50 Hz to **100 Hz** (10 ms; 16-sample ADC averaging stays).
*Builder's call: if one sample pass takes > 4 ms on hardware, reduce ADC oversampling to 8.*

**Filters** (applied per channel to calibrated force, in a pure host-tested module `fsr_filter`):

| Id | Filter | Parameter(s) | Default |
|---|---|---|---|
| 0 | None (raw) | — | |
| 1 | Moving average | N (2–20) | 5 |
| 2 | Median | N (3, 5, 7, 9) | 5 |
| 3 | Exponential (EMA) | α (0.05–1.0) | 0.3 |
| 4 | 1-D Kalman (random-walk model) | Q, R | Q 0.01, R 0.05 |

Default selected filter: **median 5** (rejects spikes, ~20 ms delay at 100 Hz). One filter is
active for all channels; parameters in `GripConfig` (append fields; bump config version with
the size-based migration already used for v3).

- The FSM, contact debounce, aggregates and force limit use the **filtered** force.
- **The force limit uses max(raw, filtered)** so filtering can never delay the safety trip.
- `/api/status` adds `fsr.forceRaw[6]` next to `fsr.force[6]` (filtered).
- FSR tab: live chart (canvas, last 10 s) of one selected channel, raw and filtered overlaid,
  with the filter selector and parameters. Readout of the filter's group delay in ms.
- Host tests: step response and spike rejection for each filter; delay figures documented.

## 6. Phase 2 — Power-up position (homing) and calibration sweep

### 6.1 Logical position frame
The firmware keeps `posOffset` (int32, RAM). **Logical position = servo position + posOffset.**
Every saved position (OPEN zero, close limit, touch reference, sweep table) and every display is
logical. Goals sent to the servo are `logical − posOffset`. After power-up `posOffset` is unknown
→ state **NOT_HOMED**: all motion except Home and jog is refused, blocker
"not homed — press Home (gripper must be empty)".

### 6.2 Home (close-to-touch)
Operator presses **Home** (Combined and Servo tabs). Preconditions: servo online, mode 0,
disarmed, no FSR reading contact. Procedure:
1. Close at **homing speed 5** with **homing current 120 units (~0.8 A)**, as repeated
   position-mode steps (like jog), until **any distal FSR ≥ contact threshold for 3 samples**
   or **stall** (position changes < 10 counts over 300 ms while commanded).
2. Torque off. Read raw position `r`.
3. Choose `k` so that `r + posOffset_k` (with `posOffset_k = k·4096`) is nearest to the saved
   **touch reference** `T` (logical). Residual = `(r + posOffset) − T`.
   - |residual| ≤ 600 counts → homed. Log the residual.
   - otherwise → homing failed (mechanics or reference changed); offer re-calibration.
4. Open to OPEN zero (now valid in the logical frame).
Timeout 60 s (full travel at speed 5 ≈ 80 s worst case is too long — **builder's call:** start
at speed 15 until 3 000 counts before the expected touch window if a coarse SEN0628 estimate is
available, else speed 10).

**First-time setup (no touch reference yet):** posOffset = 0 (frame = whatever the servo reports
now). Operator jogs to open and saves OPEN zero, then runs the **calibration sweep** (6.3) which
ends at touch and stores `T`, the close limit (`T` minus margin 100 toward open), and the table.

### 6.3 Calibration sweep
Started from the Servo tab (homed or first-time setup, gripper empty, nothing in the sensors'
view). From OPEN zero, close at speed 5 to touch (same stop condition as Home), sampling every
**100 counts** (~200 samples):
- logical position, and for each SEN0628 the 64 zone distances.
Stored in LittleFS (`/sweep.bin`, ~80 KB; the huge_app partition's data area — **builder:
confirm the partition has a filesystem, else use a custom partitions.csv**).

Derived and stored with the sweep:
- **Finger zones per position:** zones whose reading belongs to a finger (zones that change
  monotonically with position against the empty-gripper background). Used to (a) exclude fingers
  from object detection at any position, (b) estimate position from what the sensors see.
- **Fingertip opening table:** position → fingertip radius (mm from centre), computed from the
  finger zones through the sensor geometry (§4). Below ~1 000 counts (fingers invisible), linear
  extrapolation from open (170.28 mm at OPEN zero).
- Dashboard: a plot of opening vs position (thesis figure) and the finger-zone overlay per sensor.

### 6.4 Running SEN0628 position check (option C)
- **At power-up, before Home:** estimate the position by matching the current three frames to the
  sweep (nearest sample by masked zone distance, only over finger zones). Show "SEN0628 estimate:
  turn n / position p". Not used to move; shown, and used by Home to pick the start speed.
- **While homed, IDLE/MONITOR, position > 1 000 counts from open:** compare the table-predicted
  finger readings at the current position with the live frames. Disagreement above a threshold
  for > 1 s → warning + blocker "position check failed — Home again". Not evaluated while an
  object occludes finger zones (object zones detected) — builder's call on the exact rule.

## 7. Phase 3 — Object perception and object-aware grasp

### 7.1 Object points
Per frame: zones that are not finger zones (for the current position, from the sweep) and not
background (empty-gripper reading at that position, minus a margin of 15 mm) are object zones.
Convert to 3-D points with the sensor geometry and `senCal2` calibration (§4).

### 7.2 Width, class, shape
- **Width** = 2 × median horizontal distance of the object points' nearest surface to the
  gripper axis, taken per sensor and averaged over sensors that see the object (each sensor sees
  the near side; the object is roughly centred). Smoothed over 300 ms while in START.
- **Class:** small < 50 mm (egg, tube), medium 50–90 mm (apple, mug), large > 90 mm (drill).
  Thresholds configurable.
- **Shape (coarse):** *round* (three sensors give similar radii and the height profile is
  rounded), *elongated* (vertical extent ≥ 1.8 × width), *irregular* (radii differ > 25 % or
  neither of the above). **Builder's call** for exact metrics; tune on the six demo objects.
- Shown on the Combined tab: class, width mm, shape, and the object points in the 8×8 maps.

### 7.3 Grasp sequence (replaces the current PRE_TOUCH/GRASP)
States stay IDLE → START → PRE_TOUCH → GRASP → MONITOR → OPEN (+FAULT, + NOT_HOMED as a blocker):
- **START:** object present → measure width/class/shape for 300 ms (stable class required).
- **PRE_TOUCH — approach:** goal = position where fingertip opening = object radius + **x**
  (x per class, default 10 mm), at approach speed 61 with the servo's accel/decel profile (this is
  the "speed curve"). Then **slow close (speed 15)** toward the close limit until the first FSR
  contact (any enabled channel).
- **GRASP:** torque off (stop the ramp), then **slower close (speed 5)** until the class target
  force (avg of channels in contact ≥ target, ≥ minActiveFsr channels, N samples) → MONITOR.
- Existing safety (force limit, travel limit, timeouts, E-stop) unchanged.

### 7.4 Per-class profile and delicate mode
| Class | Pre-touch x | Target force | Max current |
|---|---|---|---|
| small | 10 mm | 1.0 N | 200 |
| medium | 10 mm | 2.0 N | 260 |
| large | 10 mm | 4.0 N | 350 |
| **delicate** (hidden override) | 15 mm | 0.4 N, grasp speed 3 | 150 |

*All values builder's/operator's defaults, editable.* **Delicate** is a hidden setting (Settings,
"advanced", not on the Combined tab) that, when on, replaces the class profile for the next grasp
(the egg). The detected class is still shown.

### 7.5 Contact pattern
Per finger, which FSR touched first: **proximal-first → "wrap"** (expected for large objects),
**distal-only → "pinch"** (expected for small objects in the air). Shown on the Combined tab with
a consistency mark against the size class. Display only.

## 8. Approaches considered

| Decision | Options | Chosen |
|---|---|---|
| Power-up reference | A close-to-touch; B SEN0628 recognition only; C A + B as running check | **C** (operator) |
| Finger opening vs position | A one calibration sweep with SEN0628; B known-diameter cylinders | **A** (operator, shares the sweep with C) |
| Speed curve | firmware ramp in speed mode; servo's own accel profile in position mode | **servo profile** (mode switching resets the count) |

## 9. Verification

- Host tests: filters; nearest-turn homing math (including wrap and residual limits); sweep
  table interpolation; width/class from synthetic zone frames built with the geometry; FSM
  sequence with the new sub-steps.
- Hardware checklist additions per phase (filter comparison, Home from several positions and
  after power-cycles with residuals noted, sweep plot sanity, width of each demo object).

## 10. Calls made by the designer (operator should check)
- FSR sampling 100 Hz; default filter median 5; force limit on max(raw, filtered).
- Homing speed/current 5 / 120; stall rule; residual limit 600; Home only on button press.
- Sweep step 100 counts, stored in LittleFS.
- Background margin 15 mm; shape thresholds; class profile values; delicate values.
- Slow-to-contact speed 15, grasp speed 5.
