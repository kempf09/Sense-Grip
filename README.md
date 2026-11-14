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
   servo ID stays at the saved value (default 1). There are no ID or baud fields.
4. Jog the fingers to fully open, press **Set OPEN zero here**.
5. Jog to fully closed, press **Set close limit here**.
6. Press **Open**. The gripper goes to the OPEN zero and sits in IDLE.
7. FSR tab: calibrate each channel, then Zero each one with nothing touching it.
8. Press **Arm**. Bring an object within the detect distance and it closes.

Notes:

- Saved FSR calibration (NVS `fsr402`) and the old OPEN/TOUCH references carry over from the old
  firmware. Close limit = old TOUCH minus a 100 count margin. All other old settings start from
  defaults.
- Jogging is refused while armed (disarm first). A jog stops by itself if the button is released,
  the page loses focus, or 300 ms pass without a repeat. Once both limits are set, jog stops at them.
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

## Why won't it arm?

The dashboard lists every blocker. Arm is refused while any is present.

| Text | What to do |
|---|---|
| `servo offline or feedback stale` | Check servo power and bus wiring; Servo tab, press Scan (gripper idle and clear). |
| `servo status: ...` | The servo reports voltage, temperature or overload. Fix the cause, power-cycle if needed. |
| `servo multi-turn (full-angle) feedback off` | Servo setting; needed for position across several turns. Enable it on the servo. |
| `OPEN zero / close limit not set` | Do first-boot steps 4 and 5. |
| `close limit is not on the closing side of OPEN zero` | Set the limits again with the fingers jogged the right way round. |
| `not at OPEN — press Open once after power-up` | Press Open. Also needed after every FAULT. |
| `FSR n not calibrated` | Calibrate channel n on the FSR tab. |
| `FSR n loaded at rest` | Something is pressing on pad n. Remove it, or Zero the channel. |
| `no ToF sensor online` | Check I2C wiring and sensor power. |
| `not armed` | The only one left? Press Arm. |

## API

All under the dashboard host. POST unless noted.

| Route | Purpose |
|---|---|
| GET `/` | Dashboard page |
| GET `/api/status` | State, reason, blockers, servo, FSR values |
| GET `/api/tof` | ToF readings |
| GET / POST `/api/config` | Read / save settings |
| GET / POST `/api/fsr/cal` | Read / save FSR calibration |
| `/api/fsr/zero` | Zero a calibrated FSR channel |
| `/api/servo/jog` | Jog the fingers (disarmed only) |
| `/api/servo/stop` | Stop jogging |
| `/api/servo/setOpen` | Save current position as OPEN zero |
| `/api/servo/setClose` | Save current position as close limit |
| `/api/servo/scan` | Find servo baud rate |
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
