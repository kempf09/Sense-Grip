# SENSE-GRIP

Firmware for an ESP32-S3 three-finger gripper. A CF35-12 serial servo opens and closes the
fingers, six FSR pads measure grip force, and three ToF sensors see an approaching object.
A small state machine closes on the object and holds it. You drive it from a web dashboard.
Whenever it will not do something, the dashboard tells you why.

## Wiring

| Signal | ESP32-S3 pin |
|---|---|
| FSR channels 2, 1, 3, 4, 5, 6 (in that order) | GPIO 1, 2, 4, 5, 6, 10 |
| Servo bus RX / TX | GPIO 17 / GPIO 18 |
| I2C SDA / SCL (ToF sensors, 100 kHz) | GPIO 8 / GPIO 9 |
| ToF sensor addresses | 0x33, 0x30, 0x31 |

Fingers use these channels (printed numbers): finger 1 = 6 and 2, finger 2 = 4 and 3,
finger 3 = 5 and 1. See `SenseGrip/pins.h`.

## Build and flash

Command line (same as CI):

```
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli config set library.enable_unsafe_install true
arduino-cli lib install --git-url https://github.com/DFRobot/DFRobot_MatrixLidar.git
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=32M,PartitionScheme=huge_app SenseGrip
```

Arduino IDE: open `SenseGrip/SenseGrip.ino`, install library `DFRobot_MatrixLidar`, then set
Board = ESP32S3 Dev Module, PSRAM = OPI PSRAM, Flash Size = 32 MB, Partition Scheme = Huge APP.

## First boot

1. Power the gripper. Join the Wi-Fi network `SENSE-GRIP` (password `12341234`).
2. Open http://192.168.4.1
3. Servo tab: press Scan if the servo shows offline. Scan finds the baud rate and saves it. The
   servo ID stays at the saved value (default 1). There are no ID or baud fields. Scan blocks the
   controller (and the software E-stop) for up to about 0.6 s: only scan with the gripper idle and clear.
4. Jog the fingers to fully open, press **Set OPEN zero here**.
5. **Run calibration sweep** (Servo tab, gripper empty, nothing in the SEN0628 view). It goes to
   OPEN zero, closes slowly in 100-count steps recording all three SEN0628 at each step, and stops
   when a fingertip FSR shows the tips touching. Each step that stops short (stiffer mechanism)
   is recorded where it stopped; the sweep only aborts if the jaw does not move at all, and then
   says where it stopped, how far short, and the motor current and load. That point is the **touch reference**; the close
   limit is set 100 counts before it. The sweep is saved to the ESP32's flash (`/sweep.bin`,
   LittleFS) only if it completes; it then opens. Run it again after mechanical changes.
6. FSR tab: calibrate each channel, then Zero each one with nothing touching it.
7. Press **Arm**. Bring an object within the detect distance and it closes.

### Every power-up after that: Home

The servo keeps its angle within one turn but loses the turn count at power-up, so after every
power-up (and every ESP32 restart) the dashboard shows **NOT HOMED** and refuses Open and Arm.
Empty the gripper and press **Home** (Combined or Servo tab): the fingers close slowly (speed 5,
at the Close max current setting, default 200 = 1.3 A) until a fingertip FSR shows the tips touching, the firmware picks the turn that puts
that point nearest the touch reference, and the gripper opens. Home fails, without moving
further, if the touch point is more than 600 counts from the reference (an object in the gripper?),
if the jaw stops without fingertip contact, or if the SEN0628 estimate points to a different turn.
A maximum-current push (2.7 A, at most 0.5 s, once, only in the first 0.8 s of Home and only when
all three SEN0628 are online and see no fingers) is used only for a jaw stuck past the open
position, never near closed. Home gives up after 120 s (full travel at speed 5 takes ~80 s).

OPEN zero and the close limit are saved in the same frame as the touch reference. When a lost
turn count clears them (servo power cycle, Set position mode), a successful Home re-enables
them ("homed, ... limits restored"). Setting OPEN zero or the close limit while **not** homed
starts a new frame: it is refused unless confirmed on the dashboard; confirmed, the touch reference is dropped and the calibration sweep must be run again.

**If Home keeps failing** (pads changed, mechanics changed, after Calibrate servo middle): jog to
fully open, press Set OPEN zero, and run the calibration sweep again. The sweep is only allowed
after Home or after Set OPEN zero in the same power-up, so it never uses an OPEN zero from an
earlier power-up.

- **SEN0628 estimate:** before Home, the Servo tab shows which turn the sensors think the fingers
  are on (none within ~1000 counts of open, where the fingers are not visible).
- **Running check:** while homed and idle (or holding), past the first 1000 counts and with no
  object in view, the live SEN0628 view is compared with the stored sweep at the current position.
  A mismatch for more than 1 s sets NOT HOMED ("position check failed - press Home").
- **Opening plot:** the Servo tab plots fingertip opening (mm from the axis) against motor
  position from the stored sweep; finger zones are outlined on the SEN0628 heatmaps.

Notes:

- Servo units follow the Waveshare CF library (CFServo_Arduino-250110): speed x0.732 RPM (max 61,
  about 45 RPM), acceleration x8.7 deg/s^2 (50 used), max current x6.5 mA. Defaults: approach 61,
  grasp 10, open 61.
- Moves are sent exactly as the library's WritePosEx / WriteSpe do: one write of registers 41..47.
  Writing that block starts the motion; torque enable is not written for a move.
- A stop is torque off only. At every power-up or ESP32 reset the firmware first switches torque
  off (the servo keeps running its last command across an ESP32 reset).
- **Calibrate servo middle here** (Servo tab) runs the library's CalibrationOfs: torque off, EEPROM
  unlock, calibrate (present position becomes the servo middle), EEPROM lock. All positions shift,
  so the limits are cleared: set OPEN zero and close limit again afterwards. It retries for 1 s;
  if the servo still does not answer, Last action shows `could not stop servo at boot: <error>` and
  it keeps retrying until it does.
- The servo must keep its multi-turn position across a power cycle (hardware checklist step 10).
  If it does not, the saved OPEN zero and close limit no longer match the jaw: after every power-up,
  jog to fully open and Set OPEN zero, jog to fully closed and Set close limit, then press Open.

- Saved FSR calibration (NVS `fsr402`) and the old OPEN/TOUCH references carry over from the old
  firmware. Close limit = old TOUCH minus the old safety margin (default 100 counts), backed off toward open. All other old settings start from
  defaults.
- Jogging is refused while armed (disarm first). A jog stops by itself if the button is released,
  the page loses focus, or 300 ms pass without a repeat. Once both limits are set, jog stops at them;
  tick **Ignore limits while jogging** to go past them (one jog, then it unticks itself).
- **Position mode only.** All motion (jog, approach, grasp, Open) uses servo position mode 0, and
  the firmware never changes the servo mode by itself: on this servo a mode write resets the
  position count (observed on hardware: Open switched the mode and the position jumped to ~0).
  Jog sends a goal a short way ahead and re-sends it while held; approach and grasp drive toward
  the close limit and are stopped by the FSM. The jaw cannot be back-driven, so "hold" is torque off.
- If the Servo tab shows a mode other than 0, the dashboard says so and Open/Arm are refused. Press
  **Set position mode (0)**: torque off, EEPROM unlock, mode 0, lock (saved in the servo). The
  position count may reset, so the limits are cleared: jog, then set OPEN zero and close limit again.
- **Limits after this update:** limits saved by earlier builds are cleared on first boot (they
  may have been taken before a position-count reset). Jog, set OPEN zero and close limit again.
- **Position-jump guard:** if the reported position changes by more than 400 counts within 50 ms
  (impossible for the motor), or by more than 400 counts at all while nothing is driving it (the
jaw cannot be back-driven), the limits are cleared, Open is required again, and a moving FSM
  goes to FAULT. The SEN0628 tab shows how many jumps were seen.
- **SEN0628:** initialised once and read through errors, as in Release 3.0.3; a sensor that has
  started is never restarted. A sensor whose init failed is retried every 2 s. "Online" means a
  good frame within the last second. The SEN0628 tab shows per sensor: init state and the stage
  that failed, last read code, longest read, dropouts (and how many happened while the servo was
  moving), plus the control loop's longest iteration and servo bus errors. Note these after a
  dropout: they tell wiring/EMI (dropouts while moving) from timing (long reads, slow loop).
- **Clear limits** (Servo tab) forgets OPEN zero and close limit so you can set them again.
- **FSR filtering** (FSR tab, Filter panel). FSRs are sampled at 100 Hz; one filter applies to all
  six channels: none, moving average (N 2-20), median (N 3/5/7/9), exponential (alpha 0.05-1) or a
  1-D Kalman filter (Q, R). Default: median, N = 5. The panel shows the filter's delay
  (moving average / median: (N-1)/2 x 10 ms; EMA: (1-alpha)/alpha x 10 ms; Kalman: from its
  steady-state gain) and a live 10 s chart of raw vs filtered force for one channel (the chart is
  drawn from the 5 Hz status poll). The FSM, contact detection and grasp force use the filtered
  value; the **force limit uses the higher of raw and filtered**, so filtering never delays it.
- FSR tab: untick **Used** on a channel to make the FSM ignore it (not needed for arming, never
  counts as contact or grasp force, ignored by the force limit). Readings are still shown.
  The Servo tab has its own E-STOP button, always enabled.
- After a jog, Set OPEN zero, Set close limit, or a change of "Close = positive", press Open again
  before arming.
- FSR Zero only works on a channel that is already calibrated.
- After any FAULT: press Reset (accepted only once the servo is healthy), then press Open again
  before arming.
- The dashboard keeps showing the original fault cause even after later messages.

## States

```
           Arm + object present                 any FSR contact
 IDLE ───────────────────────────► START ───► PRE_TOUCH ─────────────► GRASP
  ▲                                (checks)                             │ avg force >= target
  │                                                                      ▼  (N samples)
  │   at OPEN zero                                                    MONITOR (hold)
  └──────────── OPEN ◄──── Open button from IDLE/START/PRE_TOUCH/GRASP/MONITOR ┘
                                 FAULT ◄── any safety trip or E-stop; Reset -> IDLE
```

- IDLE: fingers held at open. Waits for Arm and an object.
- START: checks that nothing blocks, then measures the object (see below) until it has had the
  same size class for 300 ms. After 3 s without that it goes back to IDLE, disarmed, with the
  reason ("object not measured ...", "object size not stable", or a class force not below Max force).
- PRE_TOUCH: two steps. *approach*: full approach speed (servo accel profile) to the pre-touch
  position. *slow*: on arrival (or 0.4 s without progress) closing at the slow-to-contact speed
  (15) until a finger touches. No touch by the close limit, or a timeout, is a FAULT; each step
  gets the full Pre-touch timeout (on the measured sweep a 25 mm tube needs ~2.5 s + ~7 s).
  With Approach only and no pre-touch position to move to, it stops right away in MONITOR.
- GRASP: closing at the class grasp speed until the average force reaches the class target force on enough channels.
- MONITOR: holding the object. Nothing happens automatically; press Open to release.
- OPEN: moving back to the OPEN zero. On arrival (within Open tolerance, or stopped for 0.5 s within
  100 counts: the servo can stop a little short) IDLE and disarmed. The timeout message says how far it stopped.
- FAULT: torque off, with the reason shown. Reset, then Open.

Always active while moving: force limit, stale servo feedback, servo status faults, position
outside the travel limits. E-stop gives FAULT from any state. After a finished cycle the gripper is
disarmed; arm again for the next grasp.

Disarm during START, PRE_TOUCH or GRASP releases: it acts like Open (state OPEN, reason
"Disarmed during <state> — opening"). In IDLE, MONITOR, OPEN or FAULT it only clears the armed
flag; MONITOR keeps holding until you press Open.

Servo status faults come from the status register and from the status byte of every servo reply
(OR-ed together). `/api/status` shows both (`servo.status`, `servo.replyStatus`) and names the
active bits in `servo.statusText`.

### Editing the sweep

Servo tab, under the opening plot:
- **Curve points:** drag a point up or down (or select it and type a value) to correct the
  fingertip opening at that position. Edited points are orange, with the analysed value in grey;
  the pre-touch position uses the edited curve. Edits are saved in `/opening.bin` with the stored
  sweep and cleared by the next sweep. "Undo point" / "Clear all edits" go back to the analysed curve.
- **Sweep settings:** step counts per sample (default 100, 60–500), sweep speed (default 5, 1–20)
  and sweep max current (0 = Close max current). Used by the next sweep.

## Object perception and grasp (phase 3)

Needs a homed gripper and a stored calibration sweep. Every SEN0628 frame is compared with the
empty-gripper view the sweep recorded at the present finger position; zones that are not finger
zones and read closer than that background by more than max(15 mm, 20 %) are **object zones**
(cyan outlines on the heatmaps). Only points within 100 mm of the gripper axis and above the
palm count.

- **Width** = 2 × the mean, over sensors that see the object, of the median distance of its
  points from the gripper axis; averaged over 300 ms. Also shown: range (sensor distance),
  height, coarse **shape** (round / elongated / irregular / unknown with fewer than two sensors).
- **Class:** small below 50 mm, medium below 90 mm, large above (SEN0628 tab). The SEN0628 reads
  ±10 %, so an object within 10 % below a boundary takes the larger class.
- **Profile per class** (SEN0628 tab): stand-off x, target force, max current, grasp speed.
  Defaults: small 10 mm / 1 N / 200 / 5, medium 10 mm / 2 N / 260 / 5, large 10 mm / 4 N / 350 / 5,
  delicate 15 mm / 0.4 N / 150 / 3.
- **Pre-touch position:** where the fingertip opening (from the sweep) is width/2 + x + 10 % of the
  object range, so a reading 10 % short never drives the fingers into the object. Never past the
  close limit, never back toward open (an object wider than the present opening skips the approach).
- **Class override** (auto / small / medium / large), **Delicate** and **Approach only** are on the
  SEN0628 tab, RAM only (gone after a restart), and shown as badges on the Combined tab. Delicate
  beats the override. With an override or Delicate the grasp runs even if nothing is measured
  (then without the fast approach). **Approach only** stops in MONITOR at the pre-touch position:
  use it to check the stand-off with a ruler.
- **Capture background** (SEN0628 tab, gripper empty, IDLE, homed): stores the difference between
  the present view and the sweep, for a new mounting or filming location without a new sweep.
  Cleared by the next sweep.
- **Contact pattern** (Combined tab): which pad of each finger touched first. Proximal first =
  *wrap* (expected for large objects), distal only = *pinch* (small objects); ⚠ when it does not
  fit the class. Display only.
- **Max force N** must be above every class target force. The default is now 8 N; a stored older
  config keeps its value (3 N), and then large grasps are refused with that reason and config
  saves are refused until Max force (FSR tab) is raised.

## Recordings (data logging)

The control task writes one row per 10 ms step (100 Hz) into a ~5 minute ring buffer in PSRAM
(nothing is written to flash). While the dashboard is open it fetches new rows every 250 ms and
keeps them in the browser (IndexedDB, survives a page reload):

- **Per grasp, automatically:** from START until 1 s after the gripper is back in IDLE (or 1 s
  after a FAULT). Listed in the Combined tab's Recordings panel with duration, rows, class,
  width and result (grasped / aborted / FAULT).
- **Manual:** Start / Stop manual recording records everything in between (e.g. filter tests).
- **CSV:** `#` lines first (firmware build, class profile, filter, contact and max force, units),
  then `seq,t_ms,state,step,armed,pos,speed,current_mA,load_pct,f1..f6,r1..r6,c1..c3,tof_seq,
  obj_valid,width_mm,range_mm,height_mm,rad1..rad3,cls,shape,event`. Forces in N (f filtered,
  r raw, FSR1..6), position in servo counts. `event` marks state changes, the first contact per
  finger (`contact:F2:distal`) and `grasp_reached`.
- Rows are only collected while the page is open; if it was closed for more than ~5 min during a
  grasp, the recording is marked **gap**. A gripper restart closes an open recording (gap).

## Why won't it arm?

The dashboard lists every blocker. Arm is refused while any is present.

| Text | What to do |
|---|---|
| `servo offline or feedback stale` | Check servo power and bus wiring; Servo tab, press Scan (gripper idle and clear). |
| `servo status: ...` | The servo reports voltage, temperature or overload (status register or reply status byte). Fix the cause, power-cycle if needed. Reset stays refused, naming the bit, while it persists. |
| `servo multi-turn (full-angle) feedback off` | The firmware cannot change this. Set the CF35-12 to multi-turn / full-angle mode with the Waveshare servo configuration tool (the bit reads as register 18 bit 4), then power-cycle the servo. The old firmware needed the same setting, so a servo that worked with it is already set. The Servo tab shows "Full angle: NO" when it is off. Open is refused for this reason too. |
| `OPEN zero / close limit not set` | Do first-boot steps 4 and 5. |
| `close limit is not on the closing side of OPEN zero` | Set the limits again with the fingers jogged the right way round. |
| `not homed — press Home (gripper must be empty)` | Empty the gripper and press Home. First time: run the calibration sweep instead. |
| `not at OPEN — press Open (after power-up, jog, new limits)` | Press Open. Needed after power-up, after every FAULT, after jogging, after setting OPEN zero or close limit, and after changing "Close = positive". |
| `servo in mode N, not position mode 0 — ...` | Servo tab: Set position mode (0), then set the limits again. If it comes back after a servo power-cycle, the mode is not being saved: report it. |
| `FSR n not calibrated` | Calibrate channel n on the FSR tab. |
| `FSR n loaded at rest` | Something is pressing on pad n. Remove it, or Zero the channel. |
| `no ToF sensor online` | Check I2C wiring and sensor power. |
| `not armed` | Press Arm (only blocker left). |

## API

All under the dashboard host. POST unless noted.

| Route | Purpose |
|---|---|
| GET `/` | Dashboard page |
| GET `/api/status` | State, reason, blockers, servo (incl. `status`, `replyStatus`, `statusText`), FSR values |
| GET `/api/tof` | ToF readings |
| GET / POST `/api/config` | Read / save settings |
| GET / POST `/api/fsr/cal` | Read / save FSR calibration |
| `/api/fsr/zero` | Zero a calibrated FSR channel |
| `/api/servo/jog` | Jog the fingers (disarmed only) |
| `/api/servo/stop` | Stop jogging |
| `/api/servo/setOpen` | Save current position as OPEN zero |
| `/api/servo/setClose` | Save current position as close limit |
| `/api/servo/scan` | Find servo baud rate |
| `/api/servo/home` | Home: close to fingertip touch, set the position frame, open |
| `/api/servo/sweep` | Run the calibration sweep |
| GET `/api/sweep` | Stored sweep: positions, fingertip opening (effective), `auto` (analysed), `edited`, finger zones |
| GET `/api/log?from=<seq>&max=<n>` | Logged rows as CSV (headers `X-Log-Next`, `X-Log-Oldest`, `X-Log-Boot`) |
| `/api/sweep/opening?i=N&mm=X` | Edit curve point N (no `mm`: undo that point; `i=-1`: clear all edits) |
| `/api/servo/clearLimits` | Forget OPEN zero and close limit |
| `/api/servo/positionMode` | Torque off, servo mode 0 saved in EEPROM (clears limits) |
| `/api/servo/calibrateMiddle` | Servo middle calibration at the present position (clears limits) |
| `/api/fsm/arm`, `/api/fsm/disarm` | Arm / disarm |
| `/api/fsm/open` | Go to OPEN zero |
| `/api/fsm/estop` | Emergency stop, FAULT |
| `/api/fsm/reset` | Clear FAULT |
| `/api/object/override?cls=auto\|small\|medium\|large` | Class override (RAM) |
| `/api/object/delicate?on=0\|1` | Delicate mode (RAM) |
| `/api/object/approachOnly?on=0\|1` | Stop at the pre-touch position (RAM) |
| `/api/object/captureBackground` | Store the empty-gripper background correction |

`/api/status` → `object` holds the smoothed measurement (`valid`, `width`, `range`, `height`,
`shape`, `zones`), `clsDetected`, the effective `cls` (or `planWhy`), the switches, the fast
approach `goal`, the PRE_TOUCH `step`, and the contact `pattern` / `first` / `patternOk`.

`/api/status` → `fsr` contains `force` (filtered), `forceRaw`, `filterDelayMs` and `limitForce`.
Filter settings are `/api/config` fields `fsrFilter` (0 none, 1 moving average, 2 median, 3 EMA,
4 Kalman), `fsrFilterWindow`, `fsrFilterAlpha`, `fsrKalmanQ`, `fsrKalmanR`.

## Code map (`SenseGrip/`)

- `SenseGrip.ino` setup: Wi-Fi AP, starts tasks
- `pins.h` pin and address constants
- `config_core.*` settings struct, defaults, clamping, limit checks (pure)
- `config.*` saves and loads settings in NVS, migrates old values
- `fsm_core.*` state machine and blockers (pure)
- `control.*` control task: sensors + FSM + servo, owns the servo bus
- `servo_proto.*` servo packet encode/decode (pure)
- `servo.*` servo commands over UART
- `fsr_math.*` FSR force and contact maths (pure)
- `fsr_filter.*` FSR filters and their delay (pure)
- `tof_geometry.*` SEN0628 zone geometry, ported from the 3.0.3 3D page (pure)
- `homing_core.*` turn resolution, stall and unstick rules (pure)
- `sweep_core.*` finger zones, opening table (smoothed), opening → position, background, position estimate and check (pure)
- `object_core.*` object points, width, class, shape, 300 ms smoothing (pure)
- `plan_core.*` grasp plan: class, profile, pre-touch goal (pure)
- `contact_core.*` contact pattern wrap / pinch (pure)
- `log_core.*` 100 Hz data log ring and CSV rows (pure)
- `sweep_store.*` sweep file (LittleFS) and SEN0628 calibration (NVS `senCal2`)
- `fsr.*` FSR ADC reading (100 Hz), filtering and calibration storage
- `tof_detect.*` object-present detection (pure)
- `tof.*` ToF sensor reading
- `json_util.*` small JSON helpers (pure)
- `shared.*` locking for data shared between tasks
- `web.*`, `web_pages.h` HTTP routes and the dashboard page

## Host tests

```
make -C tests
```

Builds and runs the pure files (`*_core`, `*_math`, `*_proto`, `*_detect`, `*_util`, `*_filter`, `*_geometry`) with g++ on
your computer. No board needed. Hardware checks: `docs/HARDWARE_CHECKLIST.md`.
