# SENSE-GRIP Modular FSM Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single-file Release 3.0.3 sketch with a modular ESP32-S3 sketch whose FSM is IDLE → START → PRE_TOUCH → GRASP → MONITOR → OPEN (+ FAULT), closes in servo speed mode, stops on FSR readings, triggers on SEN0628 object detection after the operator arms it, and always reports why it is blocked.

**Architecture:** Hardware-free logic (FSR math, ToF detection, servo packet codec, config validation, FSM) lives in `*_core` / `*_math` / `*_proto` files with no Arduino includes and is unit-tested on the host with g++. Thin Arduino modules (`servo`, `fsr`, `tof`, `config`, `control`, `web`) own one piece of hardware each and publish snapshots. Three FreeRTOS tasks as today: control (core 1, high), tof (core 1, low), web (core 0).

**Tech Stack:** Arduino-ESP32 core 3.x, `WebServer`, `Preferences`, `Wire`, DFRobot_MatrixLidar, FreeRTOS; host tests with g++ (C++17) and a tiny in-repo check header; GitHub Actions with arduino-cli for the ESP32 compile.

**Spec:** `docs/superpowers/specs/2026-11-14-gripper-fsm-rewrite-design.md`

## Global Constraints

- Pins: servo UART1 RX 17 / TX 18; I2C SDA 8 / SCL 9 @ 100 kHz; FSR GPIO {1,2,4,5,6,10}; SEN0628 addresses {0x33,0x30,0x31}.
- FSR index order = `{1,2,4,5,6,10}`; finger map (indices) `{{5,0},{3,2},{4,1}}`; printed channel numbers `{2,1,3,4,5,6}`.
- Servo: ID 1, 1 000 000 baud default; registers: mode 33, torque 40, write block 41 = {accel u8, goal pos sign-mag15 u16, torque limit u16, goal speed u16}, goal speed 46, lock 55, feedback block 56..73 (18 bytes), phase 18 (bit 4 = full-angle/multi-turn).
- Speeds are servo command units, range 1–61 (61 ≈ 44.7 RPM).
- FSR calibration stays in NVS namespace `fsr402`, keys `s<i>r, s<i>f1, s<i>o1, s<i>f2, s<i>o2, s<i>z`; calibration valid ranges: f in [0.2, 30] N, o in (0, 10 MΩ), f1≠f2, o1≠o2, zeroMv in [0,1000].
- New config namespace `sgcfg`; servo ID/baud stay in namespace `cf35` keys `id`, `baud`.
- WiFi AP SSID `SENSE-GRIP`, password `12341234`.
- Only `servo.cpp` touches UART, only `fsr.cpp` touches the ADC, only `tof.cpp` touches I2C.
- No `.cpp` under `SenseGrip/` except Arduino glue may include `Arduino.h`; `*_core/_math/_proto` files must compile with plain g++.
- Config saves only in IDLE or FAULT.

## Review Focus

1. **Arm pressed while an object is already in front of the sensors** — expected: START happens after `detectHoldMs` of continuous presence, not instantly; covered by `fsm_arm_with_object_already_present_waits_hold` (Task 5).
2. **ToF sensor drops offline mid-detection** — expected: presence timer resets; no trigger from stale frames older than 500 ms; covered by `tof_stale_frame_not_present` (Task 3).
3. **Servo command write fails on entry to PRE_TOUCH/GRASP/OPEN** — expected: FAULT with text "servo command failed: <err>" and the motor stopped; covered by `fsm_motor_failure_faults` (Task 5).
4. **Open pressed while in FAULT** — expected: refused with reason "Reset first"; covered by `fsm_open_refused_in_fault` (Task 5).
5. **Reset pressed while the fault cause (servo status bit) persists** — expected: stays FAULT, reason names the active cause; covered by `fsm_reset_blocked_while_cause_active` (Task 5).

---

## File Structure

```
SenseGrip/
  SenseGrip.ino        setup()/loop(): load config, create queue, start tasks
  pins.h               pin + address constants (Global Constraints)
  config_core.h/.cpp   GripConfig struct, defaults, clamp/validate (host-tested)
  config.h/.cpp        NVS load/save, legacy migration
  fsr_math.h/.cpp      calibration curve, contact debounce, aggregates (host-tested)
  fsr.h/.cpp           ADC sampling, publishes FsrSnapshot
  tof_detect.h/.cpp    per-frame min distance, presence timer (host-tested)
  tof.h/.cpp           SEN0628 driver loop, publishes TofSnapshot
  servo_proto.h/.cpp   packet build/parse, sign-magnitude codecs, status-bit text (host-tested)
  servo.h/.cpp         UART, 4 verbs, mode tracking, publishes ServoSnapshot
  fsm_core.h/.cpp      Fsm class, blockers (host-tested)
  control.h/.cpp       control task: sample, poll, step FSM, apply actions, command queue
  shared.h/.cpp        portMUX + snapshot getters/setters
  web.h/.cpp           routes + JSON
  web_pages.h          embedded HTML/JS
tests/
  check.h              TEST/CHECK macros + runner
  Makefile             builds and runs test_all
  test_config.cpp test_fsr.cpp test_tof.cpp test_servo_proto.cpp test_fsm.cpp
.github/workflows/build.yml
docs/HARDWARE_CHECKLIST.md
```

---

### Task 1: Scaffold, host test harness, CI

**Files:**
- Create: `SenseGrip/SenseGrip.ino`, `SenseGrip/pins.h`, `tests/check.h`, `tests/Makefile`, `tests/test_smoke.cpp`, `.github/workflows/build.yml`

**Interfaces:**
- Produces: `tests/check.h` with `TEST(name)` (self-registering), `CHECK(cond)`, `CHECK_EQ(a,b)`, `CHECK_NEAR(a,b,eps)`; runner prints failures and exits non-zero. `make -C tests` builds every `tests/test_*.cpp` plus `SenseGrip/*_core.cpp`, `*_math.cpp`, `*_proto.cpp`, `*_detect.cpp` with `g++ -std=c++17 -Wall -Wextra -Werror -I../SenseGrip` and runs it.
- Produces: `pins.h` constants `FSR_COUNT=6`, `FSR_PINS[6]`, `FINGER_FSR_MAP[3][2]`, `FSR_CHANNEL_NUMBER[6]`, `SERVO_RX=17`, `SERVO_TX=18`, `I2C_SDA=8`, `I2C_SCL=9`, `I2C_HZ=100000`, `TOF_COUNT=3`, `TOF_ADDR[3]`.

- [ ] **Step 1:** Write `tests/test_smoke.cpp` with `TEST(smoke){ CHECK_EQ(1+1,2); }` and a deliberately failing `TEST(smoke_fail){ CHECK_EQ(1,2); }`.
- [ ] **Step 2:** Run `make -C tests` — Expected: build fails (no check.h).
- [ ] **Step 3:** Write `check.h`, `Makefile`. Run `make -C tests` — Expected: `smoke_fail` reported, exit code ≠ 0. Delete `smoke_fail`, rerun — Expected: `1 passed, 0 failed`.
- [ ] **Step 4:** Write `pins.h` and a `SenseGrip.ino` with empty `setup()`/`loop()` that includes `pins.h`.
- [ ] **Step 5:** Write `.github/workflows/build.yml`: job `host-tests` (ubuntu, `make -C tests`); job `esp32` using `arduino/setup-arduino-cli@v2`, `arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json`, `arduino-cli config set library.enable_unsafe_install true`, `arduino-cli lib install --git-url https://github.com/DFRobot/DFRobot_MatrixLidar.git`, `arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=32M,PartitionScheme=huge_app SenseGrip`.
- [ ] **Step 6:** Locally try `pip install platformio` or arduino-cli; if the network policy blocks the toolchain, record that the ESP32 compile is verified only in CI. Commit and push; check the workflow run. If the push of the workflow file is refused (missing workflow scope), keep the file, tell the operator, and continue with host tests.
- [ ] **Step 7:** Commit `chore: scaffold SenseGrip sketch, host tests and CI`.

### Task 2: Config core and FSR math

**Files:**
- Create: `SenseGrip/config_core.h/.cpp`, `SenseGrip/fsr_math.h/.cpp`, `tests/test_config.cpp`, `tests/test_fsr.cpp`

**Interfaces:**
- Produces:
```cpp
struct GripConfig {
  int32_t openZero; int32_t closeLimit; bool limitsValid; bool closePositive;
  uint16_t approachSpeed, graspSpeed, openSpeed;        // 1..61
  uint16_t closeTorqueLimit, openTorqueLimit;           // 1..430
  uint16_t openToleranceCounts;                         // 1..200, default 8
  uint16_t limitSlackCounts;                            // 0..2000, default 200
  float contactForceN; uint8_t contactSamples;          // 0.05..5 / 1..20
  float graspForceN;   uint8_t graspSamples;            // 0.1..30 / 1..20
  uint8_t minActiveFsr;                                 // 1..6
  float maxForceN;                                      // 0.5..30
  uint16_t detectMm, detectHoldMs;                      // 20..1000 / 0..2000
  uint32_t preTouchTimeoutMs, graspTimeoutMs, openTimeoutMs; // 500..60000
  bool runViaPosition;                                  // hardware-gate fallback, default false
  uint64_t tofMask[3];                                  // bit z = zone z enabled, default all ones
};
GripConfig defaultConfig();
bool clampConfig(GripConfig& c);   // clamps every field into range; returns true if anything changed
const char* limitsProblem(const GripConfig& c); // nullptr if ok, else "OPEN zero / close limit not set" or "close limit is not on the closing side of OPEN zero"

struct FsrCal { float fixedOhms, f1, o1, f2, o2, zeroMv; };
bool  fsrCalValid(const FsrCal& c);
float fsrOhms(float mv, float fixedOhms);              // 10e6 when mv<1, 0 when mv>=3299
float fsrForce(const FsrCal& c, float ohms);           // 0 when ohms<=0||>=10e6; NAN when cal invalid
struct ContactDebounce { uint8_t count[3]; bool stable[3]; };
void  updateContacts(ContactDebounce& d, const float force[6], float thresholdN, uint8_t samples);
struct FsrAggregate { float avgContactForce; uint8_t channelsInContact; float maxForce; float fingerForce[3]; };
FsrAggregate aggregate(const float force[6], float thresholdN);
```
Defaults: approach 61, grasp 10, open 61, torque 200/260, contact 0.25 N ×3, grasp 1.0 N ×3, minActiveFsr 2, maxForce 3.0 N, detect 80 mm / 200 ms, timeouts 8000/5000/15000, openZero 0, closeLimit 0, limitsValid false, closePositive true.

- [ ] **Step 1: Failing tests** (`test_config.cpp`, `test_fsr.cpp`):
  - `config_defaults_in_range`: `auto c=defaultConfig(); CHECK(!clampConfig(c));`
  - `config_clamps_speed`: `c.approachSpeed=500; CHECK(clampConfig(c)); CHECK_EQ(c.approachSpeed,61);`
  - `config_limits_problem`: limitsValid=false → non-null; openZero=1968, closeLimit=19102, closePositive=true, limitsValid=true → nullptr; closePositive=false same values → non-null.
  - `fsr_ohms_divider`: `CHECK_NEAR(fsrOhms(1650,2000),2000,1);` `CHECK_EQ(fsrOhms(0.5,2000),10e6f);`
  - `fsr_force_hits_cal_points`: cal {2000,1,30000,10,3000,0}: `fsrForce(c,30000)≈1`, `fsrForce(c,3000)≈10` (eps 0.01).
  - `fsr_contact_debounce_needs_n_samples`: finger 0 (indices 5,0) force 0.3 on index 5, threshold 0.25, samples 3 → stable false after 2 updates, true after 3; one sample below threshold resets.
  - `fsr_aggregate_counts_contact_channels`: forces {0.3,0,0,0,1.0,0} thr 0.25 → channelsInContact 2, avgContactForce 0.65, maxForce 1.0, fingerForce[2]=1.0.
- [ ] **Step 2:** `make -C tests` — Expected: FAIL (undefined symbols).
- [ ] **Step 3:** Implement. Power-law from the current code: `b=log(f2/f1)/log(o2/o1); a=f1/pow(o1,b); F=a*pow(R,b)`.
- [ ] **Step 4:** `make -C tests` — Expected: all pass.
- [ ] **Step 5:** Commit `feat: config core and FSR math`.

### Task 3: ToF detection

**Files:** Create `SenseGrip/tof_detect.h/.cpp`, `tests/test_tof.cpp`

**Interfaces:**
```cpp
uint16_t frameMinMm(const uint16_t mm[64], uint64_t mask);   // ignores 0 and >=4000 and masked zones; 0xFFFF if none
struct PresenceTimer { uint32_t sinceMs; bool active; };
// minMm[i] = frameMinMm per sensor, frameAgeMs[i] = age of that sensor's last good frame
bool updatePresence(PresenceTimer& t, const uint16_t minMm[3], const uint32_t frameAgeMs[3],
                    uint16_t detectMm, uint16_t holdMs, uint32_t nowMs);   // true when present continuously >= holdMs
```
A sensor counts only if `frameAgeMs < 500`.

- [ ] **Step 1: Failing tests:** `tof_min_ignores_invalid` (zeros and 4000 ignored; masked zone 7 with 10 mm ignored); `tof_presence_needs_hold` (min 50, detect 80, hold 200: false at t=0, false at 199, true at 200); `tof_presence_resets_on_gap` (one frame at 120 mm resets timer); `tof_any_one_sensor_triggers`; `tof_stale_frame_not_present` (age 600 → false and timer reset).
- [ ] **Step 2:** Run — FAIL. **Step 3:** Implement. **Step 4:** Run — PASS.
- [ ] **Step 5:** Commit `feat: ToF object presence detection`.

### Task 4: Servo protocol codec and servo module

**Files:** Create `SenseGrip/servo_proto.h/.cpp`, `SenseGrip/servo.h/.cpp`, `tests/test_servo_proto.cpp`

**Interfaces:**
```cpp
// servo_proto.h
enum : uint8_t { SV_PING=1, SV_READ=2, SV_WRITE=3 };
size_t  svBuildPacket(uint8_t id, uint8_t instr, const uint8_t* params, uint8_t n, uint8_t* out); // FF FF id len instr params ~sum
enum class SvParse : uint8_t { OK, NEED_MORE, BAD_CHECKSUM, WRONG_ID };
SvParse svParseReply(const uint8_t* buf, size_t len, uint8_t wantId, uint8_t& status, const uint8_t*& data, uint8_t& n, size_t& consumed);
uint16_t svEncodeSM15(int32_t v);  int32_t svDecodeSM15(uint16_t raw);
int16_t  svDecodeSM10(uint16_t raw);
void     svStatusText(uint8_t bits, char* out, size_t cap); // bit0 voltage, bit1 angle sensor, bit2 temperature, bit3 current, bit5 overload; "" when none of 0x2F
struct ServoFeedback { int32_t position; int16_t speed; float load; float voltage; uint8_t temp; uint8_t status; bool moving; int16_t current; };
ServoFeedback svDecodeFeedback(const uint8_t b[18]); // offsets as in Release 3.0.3 pollMotor()

// servo.h
struct ServoSnapshot { bool online; uint32_t ageMs; uint8_t mode; bool modeKnown; bool fullAngle; ServoFeedback fb; char lastError[48]; };
void servoBegin(uint8_t id, uint32_t baud);
bool servoPoll();                                   // reads 56..73, updates snapshot
bool servoReadModeAndPhase();                       // regs 33, 18
bool servoRunAt(int16_t speedClosingPositive, uint16_t torqueLimit);
bool servoHoldHere(uint16_t torqueLimit, uint16_t speed);
bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit);
bool servoStop();                                   // torque off
bool servoScanBaud(uint32_t& foundBaud);            // tries {1000000,500000,250000,128000,115200,76800,57600,38400}
ServoSnapshot servoGet();
void servoSetRunViaPosition(bool v, int32_t closeLimit, int32_t openZero, bool closePositive);
```
Behaviour: `servoRunAt` in speed path writes reg 33=1 only if cached mode≠1, torque on, then reg 46 = SM15(speed × closeSign) and reg 44 torque limit. In `runViaPosition` path it calls `servoMoveTo(speed>0?closeLimit:openZero, |speed|, torque)`. `servoHoldHere`/`servoMoveTo` write reg 33=0 if cached mode≠0, torque on, then block 41. Mode writes never touch reg 55. Each failure stores text in `lastError`.

- [ ] **Step 1: Failing tests:** `sv_build_ping` (id1 ping → `FF FF 01 02 01 FB`); `sv_parse_ok_and_checksum` (valid reply parses; flipped checksum → BAD_CHECKSUM; truncated → NEED_MORE; leading garbage skipped, `consumed` covers it); `sv_sm15_roundtrip` (-1234, 0, 32767); `sv_status_text` (0x04 → contains "temperature"; 0x10 → ""); `sv_decode_feedback` (hand-built 18 bytes: position −100, voltage 12.0, temp 40, moving).
- [ ] **Step 2:** Run — FAIL. **Step 3:** Implement `servo_proto.cpp`. **Step 4:** Run — PASS.
- [ ] **Step 5:** Implement `servo.cpp` on `HardwareSerial(1)` using `servo_proto` (receive loop: read available bytes with 25 ms deadline into a buffer, call `svParseReply`). No host test; covered by CI compile and Task 10 hardware checklist.
- [ ] **Step 6:** Commit `feat: CF35-12 protocol codec and servo verbs`.

### Task 5: FSM core

**Files:** Create `SenseGrip/fsm_core.h/.cpp`, `tests/test_fsm.cpp`

**Interfaces:**
```cpp
enum class State : uint8_t { IDLE, START, PRE_TOUCH, GRASP, MONITOR, OPEN, FAULT };
const char* stateName(State s);
enum class Cmd : uint8_t { NONE, ARM, DISARM, OPEN, ESTOP, RESET };
struct FsmInputs {
  uint32_t nowMs; Cmd cmd;
  bool servoOnline; uint32_t servoAgeMs; uint8_t servoStatus; bool servoFullAngle; bool servoModeKnown; int32_t position;
  bool fsrCalibrated[6]; float fsrForce[6]; bool anyFingerContact; float avgContactForce; uint8_t channelsInContact; float maxForce; uint32_t fsrSeq;
  uint8_t tofOnline; bool objectPresent;
};
struct MotorAction { enum Kind : uint8_t { NONE, RUN, HOLD, MOVE, STOP } kind; int16_t speed; int32_t position; uint16_t torque; };
struct Blocker { uint8_t id; char text[48]; };
class Fsm {
public:
  explicit Fsm(const GripConfig& c);
  void setConfig(const GripConfig& c);
  MotorAction step(const FsmInputs& in);   // at most one action per call
  void motorFailed(const char* err);       // glue calls when applying an action fails → FAULT "servo command failed: <err>"
  State state() const; bool armed() const; bool openedSinceBoot() const;
  const char* reason() const;              // last transition / refusal text
  State faultFrom() const;
  uint8_t blockers(Blocker* out, uint8_t cap) const; // computed on last step
};
```
Rules (from spec §9):
- Blockers (IDLE): servo offline (`!servoOnline || servoAgeMs>100`); servo status fault (`servoStatus&0x2F`, text from `svStatusText`); multi-turn feedback off (`!servoFullAngle`); `limitsProblem(cfg)`; "not at OPEN — press Open once after power-up" (`!openedSinceBoot`); "FSR <printed channel> not calibrated"; "FSR <ch> loaded at rest" (force ≥ contactForceN); "no ToF sensor online"; "not armed" (listed last; not a blocker for Arm itself).
- ARM: refused with reason listing the first blocker if any non-"not armed" blocker; else `armed=true`. DISARM clears it.
- IDLE + armed + objectPresent → START; START re-checks blockers: any → IDLE with reason, else → PRE_TOUCH with `RUN(+approachSpeed, closeTorqueLimit)`.
- PRE_TOUCH: anyFingerContact → GRASP with `RUN(+graspSpeed, closeTorqueLimit)`.
- GRASP: `avgContactForce>=graspForceN && channelsInContact>=minActiveFsr` for `graspSamples` consecutive new `fsrSeq` values → MONITOR with `HOLD(closeTorqueLimit, graspSpeed)`.
- PRE_TOUCH/GRASP: position past closeLimit (closing side) → FAULT "no contact before close limit" / "target force not reached before close limit"; timeouts → FAULT naming the state.
- MONITOR: no automatic action.
- OPEN cmd accepted in IDLE/START/PRE_TOUCH/GRASP/MONITOR when servo online and limits valid → OPEN with `MOVE(openZero, openSpeed, openTorqueLimit)`; refused in FAULT with "Reset first".
- OPEN: `|position-openZero|<=openToleranceCounts` → IDLE, `openedSinceBoot=true`, `armed=false`, action `HOLD(openTorqueLimit, openSpeed)`; timeout → FAULT.
- Global, every step in START..OPEN: `maxForce>=maxForceN` → FAULT "FSR force limit"; servo stale/offline → FAULT; status bits → FAULT with text; position outside `[openZero−slack, closeLimit+slack]` (oriented by closePositive) → FAULT "position outside travel limits".
- ESTOP from any state → FAULT "E-stop". Every FAULT entry emits `STOP`, records `faultFrom`, and clears `openedSinceBoot` (torque is off, so the operator must press Open again after Reset).
- RESET in FAULT: refused (stays FAULT, reason names it) while servo offline or status bits set; else → IDLE, `armed=false`.

- [ ] **Step 1: Failing tests** — helper `FsmInputs ready()` returns healthy inputs (servo online, fullAngle, all calibrated, forces 0, tofOnline 3) and helper `Fsm openedFsm()` that runs OPEN→reaches openZero. Tests:
  - `fsm_boot_blocks_until_opened`: fresh Fsm, blockers contain "press Open"; ARM refused.
  - `fsm_arm_then_trigger_runs_approach`: armed + objectPresent → START, next step → PRE_TOUCH with `RUN` speed 61.
  - `fsm_arm_with_object_already_present_waits_hold`: objectPresent is computed upstream; assert FSM does not leave IDLE while `objectPresent=false` even if armed (presence hold is Task 3's job) and enters START on the first step it is true.
  - `fsm_contact_switches_to_grasp_speed`: anyFingerContact → GRASP, `RUN` speed 10.
  - `fsm_grasp_needs_samples_and_channels`: avg 1.2 N with 1 channel never → MONITOR; with 2 channels → MONITOR after 3 distinct fsrSeq, `HOLD`.
  - `fsm_close_limit_without_contact_faults`, `fsm_pretouch_timeout_faults`.
  - `fsm_force_limit_faults_from_grasp` (maxForce 3.1 → FAULT, `STOP`, faultFrom GRASP).
  - `fsm_open_from_monitor_returns_idle_disarmed`.
  - `fsm_open_refused_in_fault`, `fsm_reset_blocked_while_cause_active`, `fsm_motor_failure_faults`, `fsm_estop_from_any_state`.
  - `fsm_uncalibrated_fsr_named`: fsrCalibrated[2]=false → blocker text "FSR 3 not calibrated" (printed channel of index 2 is 3).
- [ ] **Step 2:** `make -C tests` — FAIL. **Step 3:** Implement with one `switch(state)` in `step()`; entry actions in a private `enter(State, const char* why)` that returns the MotorAction. **Step 4:** Run — PASS.
- [ ] **Step 5:** Commit `feat: gripper FSM core with blockers`.

### Task 6: Hardware modules — shared snapshots, FSR, ToF, config NVS

**Files:** Create `SenseGrip/shared.h/.cpp`, `SenseGrip/fsr.h/.cpp`, `SenseGrip/tof.h/.cpp`, `SenseGrip/config.h/.cpp`

**Interfaces:**
```cpp
// shared.h
extern portMUX_TYPE gMux;
template<class T> T snap(const T& src);   // copies under gMux
// fsr.h
struct FsrSnapshot { uint16_t raw[6]; float mv[6], ohms[6], force[6]; bool calibrated[6]; ContactDebounce contacts; FsrAggregate agg; uint32_t seq; };
void fsrBegin(); void fsrSample(float contactN, uint8_t samples); FsrSnapshot fsrGet();
FsrCal fsrGetCal(uint8_t i); bool fsrSaveCal(uint8_t i, const FsrCal& c); bool fsrZero(uint8_t i);
// tof.h
struct TofSensor { bool online; uint16_t mm[64]; uint16_t minMm; uint32_t lastGoodMs, ok, err; };
struct TofSnapshot { TofSensor s[3]; uint8_t online; bool objectPresent; };
void tofTask(void*); TofSnapshot tofGet(); void tofSetDetect(uint16_t mm, uint16_t holdMs, const uint64_t mask[3]);
// config.h
GripConfig configLoad();            // sgcfg, else legacy migration, else defaults; always clamped
bool configSave(const GripConfig&); void servoIdBaudLoad(uint8_t& id, uint32_t& baud); bool servoIdBaudSave(uint8_t, uint32_t);
```
- Legacy migration: if `sgcfg` has no `cfg` blob, read namespace `fsm` key `cfg` into a private copy of the Release 3.0.3 `FsmConfig` layout (copy the struct verbatim from the old `.ino` lines 114–127) when its size matches and `magic==0x46534D34`; take `openZero`, `closePositive`, and `closeLimit = touchPosition − sign×travelSafetyMarginCounts`, `limitsValid = openZeroValid && touchPositionValid`; save to `sgcfg`. Never write to the old namespaces.
- FSR sampling: 16-sample average of `analogReadMilliVolts`, 12-bit, 11 dB attenuation, minus `zeroMv`, then `fsr_math`.
- ToF task: `Wire.begin(8,9,100000)`, `setTimeOut(120)`, init each lidar in 8×8 mode, round-robin `getAllData`, `vTaskDelay(1)` between sensors, run `frameMinMm` + `updatePresence`. Retry init of an offline sensor every 2 s.

- [ ] **Step 1:** Implement the four modules.
- [ ] **Step 2:** `make -C tests` still passes (host build excludes these).
- [ ] **Step 3:** Push; CI `esp32` job — Expected: compiles (warnings allowed, no errors).
- [ ] **Step 4:** Commit `feat: FSR, ToF and config hardware modules`.

### Task 7: Control task wiring

**Files:** Create `SenseGrip/control.h/.cpp`; Modify `SenseGrip/SenseGrip.ino`

**Interfaces:**
```cpp
struct ControlRequest { enum Kind : uint8_t { FSM_CMD, JOG, JOG_STOP, SET_OPEN, SET_CLOSE, SCAN, CONFIG } kind; Cmd cmd; int16_t speed; GripConfig cfg; };
bool controlEnqueue(const ControlRequest& r);   // xQueueSend, 0 wait; false when full
struct ControlStatus { State state; State faultFrom; bool armed; bool openedSinceBoot; char reason[96]; Blocker blockers[10]; uint8_t nBlockers; char lastActionMsg[96]; bool lastActionOk; };
ControlStatus controlGet();
void controlTask(void*);
```
Loop every 1 ms tick: drain queue; every 10 ms `servoPoll()`; every 20 ms `fsrSample()`; every 1000 ms `servoReadModeAndPhase()`; build `FsmInputs` from snapshots; `fsm.step()`; apply returned `MotorAction` (RUN→`servoRunAt`, HOLD→`servoHoldHere`, MOVE→`servoMoveTo`, STOP→`servoStop`), on false call `fsm.motorFailed(servoGet().lastError)` and `servoStop()`.
- Jog/SET_OPEN/SET_CLOSE/SCAN/CONFIG accepted only in IDLE or FAULT; otherwise `lastActionMsg="<action> only allowed in IDLE or FAULT (now <state>)"`. JOG = `servoRunAt(speed)` with auto JOG_STOP if no repeat request within 300 ms (dead-man), and refused past soft limits once limits are valid. SET_OPEN stores current position as openZero; SET_CLOSE as closeLimit; both save config and call `fsm.setConfig`.
- `setup()`: Serial 115200, `configLoad`, `servoBegin`, create queue (8), `WiFi.softAP("SENSE-GRIP","12341234")`, `webBegin()`, create tasks (control core1 prio3 8 KB; tof core1 prio1 8 KB; web core0 prio1 12 KB). Boot: no motion commanded.

- [ ] **Step 1:** Implement. **Step 2:** CI compile — PASS. **Step 3:** Commit `feat: control task and setup wiring`.

### Task 8: Web JSON API

**Files:** Create `SenseGrip/web.h/.cpp`

**Interfaces:** `void webBegin(); void webTask(void*);` Routes (all JSON, `Cache-Control: no-store`, errors `{"ok":false,"error":"..."}` with 4xx):
- `GET /api/status` → `{ok, state, faultFrom, armed, openedSinceBoot, reason, blockers:[text], lastAction:{msg,ok}, servo:{online,ageMs,position,speed,load,current,voltage,temp,status,statusText,mode,fullAngle}, fsr:{force[6],raw[6],mv[6],ohms[6],calibrated[6],fingerForce[3],fingerContact[3],avgContactForce,channelsInContact,maxForce}, tof:{online,objectPresent,minMm[3]}, limits:{openZero,closeLimit,valid}}`
- `GET /api/tof` → `{sensors:[{online,ok,err,ageMs,minMm,mm:[64]}×3]}`
- `GET /api/config`, `POST /api/config` (form fields named as `GripConfig` members; unknown/invalid → 400 naming the field; queued as CONFIG)
- `GET /api/fsr/cal`, `POST /api/fsr/cal` (`id,r,f1,o1,f2,o2,z`), `POST /api/fsr/zero` (`id`) — IDLE/FAULT only, 409 otherwise
- `POST /api/servo/jog` (`speed`, signed, |speed|≤61), `/api/servo/stop`, `/api/servo/setOpen`, `/api/servo/setClose`, `/api/servo/scan`
- `POST /api/fsm/arm|disarm|open|estop|reset`
- Queue full → 503 `"controller busy"`. E-stop additionally sets a flag the control loop checks first.

- [ ] **Step 1:** Implement with `snprintf` into a static buffer (status JSON < 3 KB; tof JSON built with `server.setContentLength(CONTENT_LENGTH_UNKNOWN)` + `sendContent` chunks).
- [ ] **Step 2:** Add a host test `web_json_escape` only if you write an escape helper (`jsonEscape(const char*, char*, size_t)` in `servo_proto`-style pure file `json_util.h/.cpp`); reason strings must be escaped.
- [ ] **Step 3:** CI compile — PASS. **Step 4:** Commit `feat: JSON API`.

### Task 9: Dashboards

**Files:** Create `SenseGrip/web_pages.h` (one `PROGMEM` raw string), route `GET /` in `web.cpp`

Tabs: **Combined** (default), **Servo**, **FSR**, **SEN0628**. Polling: `/api/status` every 200 ms; `/api/tof` every 200 ms only while Combined or SEN0628 is visible. Vanilla JS, no CDN (AP has no internet).
- Combined: state badge (colour per state; FAULT red with `faultFrom` and reason), Armed toggle, blockers checklist (each blocker a red line; "Ready" green when only "not armed"), three finger force bars with contact dots, position bar between openZero and closeLimit, three 8×8 heatmaps (canvas, 0–400 mm colour scale, min distance printed), buttons Arm/Disarm, Open, E-STOP (large), Reset.
- Servo: telemetry table, ID/baud + Scan, press-and-hold Jog Close/Jog Open (repeat POST every 150 ms while held; release sends stop), speed input 1–61, Set OPEN zero here, Set close limit here, speed/torque/timeout config fields, `runViaPosition` checkbox.
- FSR: six rows (printed channel, finger, raw, mV, Ω, N, calibrated), per-row Zero and calibration form; contact/grasp/max force + samples + minActiveFsr fields.
- SEN0628: three heatmaps with mm numbers per cell, health counters, detectMm/detectHoldMs, click-to-toggle zone mask per sensor (saved via config).
- Every POST shows the API's `error` text in a toast; disabled buttons show the first blocker as tooltip.

- [ ] **Step 1:** Write page. **Step 2:** Extract the `<script>` body and run `node --check` on it — Expected: no syntax errors. **Step 3:** CI compile — PASS. **Step 4:** Commit `feat: dashboards`.

### Task 10: Remove old release, docs, hardware checklist

**Files:** Delete `SENSE_GRIP_Release_3_0_3_Simple_FSM.ino`, `SENSE_GRIP_Release_3_0_3_Types.h`, `RELEASE_3_0_3_README.txt`, `CONTINUOUS_APPROACH.md`. Create `README.md`, `docs/HARDWARE_CHECKLIST.md`.

- README: what it is, wiring table, build (arduino-cli line from CI), first-boot steps, state diagram from spec §9, API list.
- HARDWARE_CHECKLIST (operator runs, gripper unloaded, hand on E-stop):
  1. Servo gate (spec §6): Servo tab Jog at speed 10 → fingers move, `mode` reads 1; release → stops. Power-cycle → `mode` reads the pre-jog value (proves mode write is not persisted). After a jog, Open → no jump on the 1→0 switch. Any failure → tick `runViaPosition`.
  2. Set OPEN zero / close limit; Open → IDLE shows only "not armed".
  3. Arm with nothing in front; wave hand at < 80 mm for > 200 ms → closes; touch an FSR → slows; press it ≥ 1 N on 2 channels → MONITOR.
  4. No object: arm + trigger → FAULT at close limit with reason.
  5. E-stop in PRE_TOUCH, GRASP, OPEN → FAULT, torque off.
  6. Grasp a soft object; Open → IDLE, disarmed.
- [ ] **Step 1:** Write docs, delete files. **Step 2:** `make -C tests` PASS and CI green. **Step 3:** Commit `chore: remove Release 3.0.3 files, add README and hardware checklist`; push.
