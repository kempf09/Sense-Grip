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
5. Jog to fully closed, press **Set close limit here**.
6. Press **Open**. The gripper goes to the OPEN zero and sits in IDLE. Open is refused while any servo blocker shows, including "Full angle: NO" (see "Why won't it arm?").
   Press Open again after jogging or setting limits: arming needs the jaw to have reached OPEN since then.
7. FSR tab: calibrate each channel, then Zero each one with nothing touching it.
8. Press **Arm**. Bring an object within the detect distance and it closes.

Notes:

- Speeds are in servo steps per second (4096 steps = one turn; 2000 steps/s is about 29 RPM).
  Defaults: approach 2000, grasp 300, open 2000; maximum 3400. Settings saved by earlier builds
  (which capped speed at 61, about 1 RPM) get these new speeds on first boot; limits are kept.
- At every power-up or ESP32 reset the firmware first sets the servo torque limit to 0 and switches
  torque off (the servo keeps running its last command across an ESP32 reset). A stop never writes
  a goal speed or position: on this servo that re-enables torque. It retries for 1 s;
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
  tick **Ignore limits while jogging** to go past them (one jog, then it unticks itself). Not
  available with "Run via position".
- **Clear limits** (Servo tab) forgets OPEN zero and close limit so you can set them again.
- **Set position mode (0)** (Servo tab) switches torque off and puts the servo in position mode.
  Use it if the Servo tab shows a mode other than 0 while idle.
- A ToF sensor with read errors keeps its last frame for up to 500 ms (ignored for triggering after
  that). It is only restarted after 5 s without a good frame, and only shown offline when it also
  stops answering on the I2C bus.
  The Servo tab has its own E-STOP button, always enabled.
- After a jog, Set OPEN zero, Set close limit, or a change of "Close = positive" or "Run via
  position", press Open again before arming.
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
- START: one-cycle check that nothing blocks. If something does, back to IDLE with the reason.
- PRE_TOUCH: closing at approach speed until a finger touches. No touch by the close limit, or a timeout, is a FAULT.
- GRASP: closing slowly until the average force reaches the target on enough channels.
- MONITOR: holding the object. Nothing happens automatically; press Open to release.
- OPEN: moving back to the OPEN zero. On arrival: IDLE and disarmed.
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

## Why won't it arm?

The dashboard lists every blocker. Arm is refused while any is present.

| Text | What to do |
|---|---|
| `servo offline or feedback stale` | Check servo power and bus wiring; Servo tab, press Scan (gripper idle and clear). |
| `servo status: ...` | The servo reports voltage, temperature or overload (status register or reply status byte). Fix the cause, power-cycle if needed. Reset stays refused, naming the bit, while it persists. |
| `servo multi-turn (full-angle) feedback off` | The firmware cannot change this. Set the CF35-12 to multi-turn / full-angle mode with the Waveshare servo configuration tool (the bit reads as register 18 bit 4), then power-cycle the servo. The old firmware needed the same setting, so a servo that worked with it is already set. The Servo tab shows "Full angle: NO" when it is off. Open is refused for this reason too. |
| `OPEN zero / close limit not set` | Do first-boot steps 4 and 5. |
| `close limit is not on the closing side of OPEN zero` | Set the limits again with the fingers jogged the right way round. |
| `not at OPEN — press Open (after power-up, jog, new limits)` | Press Open. Needed after power-up, after every FAULT, after jogging, after setting OPEN zero or close limit, and after changing "Close = positive" or "Run via position". |
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
| `/api/servo/clearLimits` | Forget OPEN zero and close limit |
| `/api/servo/positionMode` | Torque off, servo mode 0 |
| `/api/fsm/arm`, `/api/fsm/disarm` | Arm / disarm |
| `/api/fsm/open` | Go to OPEN zero |
| `/api/fsm/estop` | Emergency stop, FAULT |
| `/api/fsm/reset` | Clear FAULT |

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
- `fsr.*` FSR ADC reading and calibration storage
- `tof_detect.*` object-present detection (pure)
- `tof.*` ToF sensor reading
- `json_util.*` small JSON helpers (pure)
- `shared.*` locking for data shared between tasks
- `web.*`, `web_pages.h` HTTP routes and the dashboard page

## Host tests

```
make -C tests
```

Builds and runs the pure files (`*_core`, `*_math`, `*_proto`, `*_detect`, `*_util`) with g++ on
your computer. No board needed. Hardware checks: `docs/HARDWARE_CHECKLIST.md`.
