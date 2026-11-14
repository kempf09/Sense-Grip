# SENSE-GRIP firmware rewrite — design

Date: 2026-11-14
Branch: `feature/continuous-approach-touch-window`
Status: draft for approval

## 1. Why

The current firmware (`SENSE_GRIP_Release_3_0_3_Simple_FSM.ino`, 1,645 lines in one file)
does not match the intended gripper behaviour and is hard to follow:

- It is spaghetti: FSM, servo protocol, safety locks, NVS migration and two embedded web
  apps are interleaved in one file.
- Servo setup and motion are slow. Closing is either a drive to a saved TOUCH encoder
  position or a stream of 10–50-count position steps.
- When it refuses to move, the operator often cannot see why.
- The SEN0628 ToF sensors play no role in the FSM.

The goal is a **working, readable FSM** for a three-finger underactuated gripper that
follows the operator's state model, with motion driven by sensors (ToF to start, FSR to
stop), not by saved encoder targets.

## 2. Hardware (unchanged)

| Part | Connection |
|---|---|
| ESP32-S3-WROOM-2 N32R16V | — |
| Waveshare CF35-12 servo (Feetech-style half-duplex bus, ID 1, 1 Mbaud default) | UART1 RX 17, TX 18 |
| 3 × SEN0628 8×8 ToF (DFRobot_MatrixLidar) | I2C SDA 8, SCL 9, 100 kHz, addresses 0x33, 0x30, 0x31 |
| 6 × FSR402, 2 kΩ divider resistors | ADC GPIO 1, 2, 4, 5, 6, 10 |

Finger-to-FSR mapping is kept from the current code: finger 1 = GPIO10 tip + GPIO1 base,
finger 2 = GPIO5 tip + GPIO4 base, finger 3 = GPIO6 tip + GPIO2 base.

## 3. Goals, non-goals, anti-goals

**Goals**
- Six operator states, each readable on one screen of code.
- Fast reaction: the motor stops within one control cycle (≤ 20 ms) of an FSR contact.
- Every blocked action reports a named reason, visible on the dashboard.
- Web dashboards via a JSON API: Servo, FSR, SEN0628 (8×8 matrix per sensor), Combined.

**Non-goals (this round)**
- MONITOR logic (slip detection with ToF). The state exists as a holding placeholder.
- The SEN0628 3D point-cloud view and plane-fit calibration page. Not carried over
  (builder's call: not in the requested dashboard list; can return later as its own page).
- Automatic re-grasp or retries.

**Anti-goals (failure even if it "works")**
- The gripper closing when it has not been armed.
- A blocked or faulted state with no reason shown.
- Logic spread back across many files and globals so a state can't be read in one place.

## 4. Approaches considered

| | Approach | Trade-off |
|---|---|---|
| **A (chosen)** | Modular rewrite; closing driven in speed mode, holding/opening in position mode | Cleanest match to "sensors decide when to stop". CF35-12 speed-mode behaviour must be verified on hardware. Loses the saved-TOUCH approach. |
| B | Modular rewrite; keep position mode with a far target, cancel on contact | Avoids mode switching but keeps the multi-turn full-angle dependency that caused most start-up lockouts. |
| C | Refactor the existing file in place | Cheapest; most of the tangle stays. |

The operator chose **A**.

## 5. Architecture

Arduino sketch folder, one `.ino` plus modules:

```
SenseGrip/
  SenseGrip.ino        setup(): load config, start tasks. loop(): idle.
  config.h/.cpp        Pins, constants, Config struct, NVS load/save.
  servo.h/.cpp         CF35-12 bus protocol + 3-verb motion API + telemetry.
  fsr.h/.cpp           ADC sampling, calibration curve, per-finger contact, force.
  tof.h/.cpp           SEN0628 init/read, object-present detection.
  fsm.h/.cpp           State machine, guards ("blockers"), fault handling.
  web.h/.cpp           Routes + JSON API.
  web_pages.h          Embedded HTML/JS for the four dashboards.
```

Rule: each module owns its hardware and its data. Other modules read **snapshots**
(copied structs) through getters; nothing outside `servo.cpp` touches the UART, nothing
outside `fsr.cpp` touches the ADC, nothing outside `tof.cpp` touches I2C.

### Tasks (kept from the current design, which works)

| Task | Core | Priority | Period | Does |
|---|---|---|---|---|
| control | 1 | high | 10 ms loop | poll servo, sample FSR (every 20 ms), run FSM, drain command queue |
| tof | 1 | low | as fast as sensors allow (~15 Hz × 3) | read SEN0628, publish frames |
| web | 0 | normal | 2 ms | `server.handleClient()` |

Web → control commands go through a FreeRTOS queue (as today). Snapshots are guarded
by one `portMUX` critical section (as today).

### Source layout

The new sketch lives in `SenseGrip/` at the repo root. The old `SENSE_GRIP_Release_3_0_3_*`
files and docs are deleted once the new sketch compiles (git history keeps them).
(Builder's call.)

## 6. Servo module

Three verbs, plus stop:

```cpp
bool servoRunAt(int16_t speed, uint16_t torqueLimit);   // signed, + = closing
bool servoHoldHere(uint16_t torqueLimit);                // position hold at current encoder count
bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit);
bool servoStop();                                       // torque off
ServoSnapshot servoGet();  // position, speed, load, current, voltage, temp, status, moving, ok, ageMs
```

Register usage taken from the existing working code: mode reg 33, torque enable 40,
write block 41 = {accel, goal pos (sign-mag 15), torque limit, goal speed}, feedback
block 56..73. Speed in speed mode = goal speed register (46) sign-magnitude.

**Mode handling.** The module tracks the servo's current mode and only writes reg 33 when
a verb needs a different mode.

**Hardware-verification gate (first build task):** confirm on the CF35-12 that
(1) mode 1 runs at a commanded signed speed, (2) a mode write does not persist to EEPROM
on every change, or EEPROM lock (reg 55) keeps it in RAM, (3) switching 1→0 holds the
current position without a jump. **If any fails, `servoRunAt` is implemented in position
mode** as `moveTo(softLimit, |speed|)`; the FSM does not change.

**Soft travel limits.** `openZero` and `closeLimit` (encoder counts) are config.
`servoRunAt` closing is refused past `closeLimit`; the control loop stops the motor if
position crosses either limit. This replaces the TOUCH reference and the
multi-turn/full-angle start-up lockout.

**Servo setup** becomes: ping/scan baud, read mode, jog open/close at a chosen speed
while held (press-and-hold buttons send run/stop), "Set OPEN zero here", "Set close limit
here". No safe-shutdown marker, no reference-verification lock: on boot the operator
presses "Open" (move to openZero) once before arming. (Builder's call, flagged for
operator review — see §12.)

## 7. FSR module

Kept from current code: 16-sample ADC averaging, divider → ohms, two-point power-law
calibration, zero offset, NVS namespace `fsr402` and its key names (existing calibration
loads unchanged). Published per sample: raw, mV, ohms, force per channel; per-finger force
(max of its two FSRs); per-finger contact (force ≥ `contactForceN` for `contactSamples`
consecutive samples); `anyContact`; `avgForce` over active channels; `maxForce`.

## 8. ToF module

Kept: three DFRobot_MatrixLidar instances, 8×8 mode, round-robin reads, per-sensor
health (found, ok/error counts, last-good age).

New: **object detection** per sensor = minimum of the 64 zones, ignoring zones flagged
invalid (0 or ≥ 4000 mm) and zones in an optional per-sensor mask (builder's call: mask
editable from SEN0628 dashboard, stored in NVS, default all on). `objectPresent` = any
one sensor's minimum < `detectMm` continuously for `detectHoldMs`.

## 9. FSM

States: `IDLE, START, PRE_TOUCH, GRASP, MONITOR, OPEN, FAULT`.
Plus a flag `armed` (operator-controlled).

```
           Arm + objectPresent(hold)            any FSR contact
 IDLE ───────────────────────────► START ───► PRE_TOUCH ─────────────► GRASP
  ▲                                (checks,                            │ avgForce ≥ target
  │                                 enter)                             ▼  (N samples)
  │   at openZero                                                    MONITOR (hold)
  └──────────── OPEN ◄──── "Open" button from IDLE/START/PRE_TOUCH/GRASP/MONITOR ┘
                                 FAULT ◄── any safety trip; Reset → IDLE
```

| State | Entry | Each cycle | Exit |
|---|---|---|---|
| IDLE | servoHoldHere (keeps fingers at open) | read sensors, compute blockers | `armed && objectPresent` → START |
| START | — | verify no blockers; one cycle | ok → PRE_TOUCH; blocker → IDLE with reason |
| PRE_TOUCH | `servoRunAt(+approachSpeed)` | watch FSR | any finger contact → GRASP; closeLimit reached → FAULT "no contact before close limit"; timeout → FAULT |
| GRASP | `servoRunAt(+graspSpeed)` | watch avg force | avgForce ≥ target with ≥ minActiveFsr channels above contact threshold, for N samples → MONITOR; closeLimit/timeout → FAULT |
| MONITOR | `servoHoldHere` | (placeholder) show force; no automatic action | Open button → OPEN |
| OPEN | `servoMoveTo(openZero, openSpeed)` | wait | within tolerance → IDLE (disarmed); timeout → FAULT |
| FAULT | `servoStop()` (torque off) | — | Reset (only when fault cause cleared) → IDLE |

After a cycle completes, `armed` is cleared: the operator re-arms for the next grasp
(builder's call; prevents immediate re-grasp of the released object).

**Global safety checks, every cycle while moving:** any FSR ≥ `maxForceN` → FAULT;
servo feedback stale > 100 ms → FAULT; servo status fault bits → FAULT; position outside
soft limits → FAULT. E-stop button → FAULT from any state.

### Blockers ("why won't it go")

`fsm.cpp` exposes `blockers()` → a list of `{id, text}` evaluated every cycle in IDLE:

- servo offline / wrong baud
- servo status fault (bits decoded to text: voltage, temperature, overload, …)
- OPEN zero / close limit not set, or close limit ≤ open zero
- not at OPEN position (must press Open once after boot)
- FSR channel *n* not calibrated
- FSR channel *n* reading ≥ contact threshold at rest (pre-loaded)
- no ToF sensor online
- not armed

"Arm" is refused while any other blocker is present, and the response lists them.
FAULT always carries a reason string and the state it faulted from.

## 10. Config (NVS namespace `sgcfg`, new)

| Key | Default | Range |
|---|---|---|
| openZero, closeLimit | from old `fsm` namespace if present (1968 / 19202 − 100) else unset | ±32767 |
| approachSpeed | 61 (servo command units, current documented max ≈ 44.7 RPM) | 1–61 until the hardware gate shows a higher valid max |
| graspSpeed | 10 | 1–61 |
| openSpeed | 61 | 1–61 |
| minActiveFsr | 2 | 1–6 |
| closeTorqueLimit, openTorqueLimit | 200, 260 (from current code) | 1–430 |
| contactForceN / contactSamples | 0.25 N / 3 | |
| graspForceN / graspSamples | 1.0 N / 3 | |
| maxForceN | 3.0 N | 0.5–30 |
| detectMm / detectHoldMs | 80 mm / 200 ms | 20–1000 / 0–2000 |
| preTouchTimeoutMs, graspTimeoutMs, openTimeoutMs | 8000, 5000, 15000 | |
| servoId, servoBaud | 1, 1000000 | |

Speed units: the builder confirms the CF35-12 speed unit during the hardware gate and
labels the UI accordingly. Config saves only in IDLE/FAULT.

## 11. Web API and dashboards

WiFi: AP `SENSE-GRIP` (kept). Endpoints (JSON):

- `GET /api/status` — state, armed, reason, blockers[], servo snapshot, FSR snapshot,
  ToF summary (per-sensor min mm, online), objectPresent.
- `GET /api/tof` — three 8×8 arrays + health.
- `GET/POST /api/config`
- `GET/POST /api/fsr/cal`, `POST /api/fsr/zero`
- `POST /api/servo/{jog,stop,setOpen,setClose,scan}`
- `POST /api/fsm/{arm,disarm,open,estop,reset}`

Pages (one embedded HTML app, tabbed, polling `/api/status` ~5 Hz, `/api/tof` ~5 Hz
when its tab is open):

1. **Combined** — big state indicator, armed, blockers checklist, per-finger force bars,
   servo position bar between open/close limits, three small 8×8 heatmaps, buttons
   Arm / Open / E-stop / Reset.
2. **Servo** — config + telemetry, jog, set limits, scan.
3. **FSR** — six channels: raw, mV, ohms, N; calibration form; zero.
4. **SEN0628** — three 8×8 heatmaps with mm values, health, detect settings, zone mask.

## 12. Calls made by the designer (operator should check)

- Arm then auto-trigger; any one ToF sensor triggers. *(operator decided)*
- Speed-mode closing behind a 3-verb servo API with position-mode fallback. *(operator chose A)*
- 3D view and plane-fit calibration dropped.
- Safe-shutdown marker / boot reference lock removed; replaced by "press Open once after boot".
- Grasp completes when the *average* force of channels in contact reaches the target and at least 2 channels are in contact; MONITOR holds until Open is pressed.
- IDLE keeps torque on, holding the fingers at open.
- Speeds stay in the servo's own command units (max 61 ≈ 44.7 RPM, from the current notes).
- New sketch in `SenseGrip/`; old release files removed.
- Arm clears after each cycle.
- Old FSR calibration is kept; old FSM settings are not migrated except openZero/closeLimit.

## 13. Verification

- Compile with arduino-cli for ESP32-S3 (esp32 core 3.x) in CI/locally.
- Host-side unit tests (PlatformIO `native` or plain g++) for FSM transitions using
  fake servo/FSR/ToF snapshots, FSR calibration math, blocker list.
- Hardware checklist: servo gate (§6), jog/limits, FSR contact stop with no object →
  fault at close limit, grasp of a soft test object, E-stop during each moving state.
