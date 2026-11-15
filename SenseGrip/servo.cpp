#include "servo.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include "pins.h"

namespace {
HardwareSerial Bus(1);
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint32_t REPLY_TIMEOUT_US = 25000;
constexpr uint8_t  ACCEL = 50;          // units of 8.7 deg/s^2, as in the Waveshare CF examples
constexpr size_t   RX_CAP = 64;

int32_t  posOffset = 0;          // logical = servo + posOffset (RAM; homing sets it)
uint8_t  servoId = 1;
uint32_t servoBaud = 1000000;

// Shared state: only touched inside mux (except UART-task-only config below).
ServoSnapshot snap = {};
uint32_t lastGoodMs = 0;
bool     everGood = false;
SvPhaseLatch phaseLatch = {false, 0};   // UART-task-only

// UART-task-only.
int32_t closeLimitPos = 0, openZeroPos = 0;

void setError(const char* text) {
  portENTER_CRITICAL(&mux);
  strncpy(snap.lastError, text, sizeof(snap.lastError) - 1);
  snap.lastError[sizeof(snap.lastError) - 1] = '\0';
  portEXIT_CRITICAL(&mux);
}

void clearError() {
  portENTER_CRITICAL(&mux); snap.lastError[0] = '\0'; portEXIT_CRITICAL(&mux);
}

void getError(char* out, size_t cap) {
  portENTER_CRITICAL(&mux);
  strncpy(out, snap.lastError, cap - 1);
  out[cap - 1] = '\0';
  portEXIT_CRITICAL(&mux);
}

void busBegin(uint32_t baud) {
  servoBaud = baud;
  Bus.end();
  delay(15);
  Bus.begin(baud, SERIAL_8N1, SERVO_RX, SERVO_TX);
  delay(25);
  while (Bus.available()) Bus.read();
}

// Sends one packet and waits for the reply to it. On success returns true with
// status and (copied) params. On failure sets lastError.
bool transact(uint8_t instr, const uint8_t* params, uint8_t n, uint8_t& status,
              uint8_t* out, uint8_t& outN) {
  uint8_t tx[24];
  if (n > sizeof(tx) - 6) { setError("request too long"); return false; }
  const size_t txLen = svBuildPacket(servoId, instr, params, n, tx);
  while (Bus.available()) Bus.read();
  Bus.write(tx, txLen);
  Bus.flush();

  uint8_t rx[RX_CAP];
  size_t have = 0;
  bool sawBadSum = false, sawWrongId = false;
  const uint32_t start = micros();
  while ((uint32_t)(micros() - start) < REPLY_TIMEOUT_US) {
    bool got = false;
    while (Bus.available() && have < RX_CAP) { rx[have++] = (uint8_t)Bus.read(); got = true; }
    // Sleep (not taskYIELD): a yield never lets lower-priority tasks run, so a servo that
    // stops answering would starve the SEN0628 task for the whole 25 ms timeout.
    if (!got) { vTaskDelay(1); continue; }
    for (;;) {
      const uint8_t* d = nullptr; uint8_t dn = 0, st = 0; size_t used = 0;
      SvParse r = svParseReply(rx, have, servoId, st, d, dn, used);
      if (r == SvParse::OK) {
        status = st; outN = dn;
        // The status byte carries the servo's error bits; it is data, not a bus failure.
        portENTER_CRITICAL(&mux); snap.replyStatus = st; portEXIT_CRITICAL(&mux);
        if (out && dn) memcpy(out, d, dn);
        return true;
      }
      if (r == SvParse::BAD_CHECKSUM) sawBadSum = true;
      if (r == SvParse::WRONG_ID) sawWrongId = true;
      if (used == 0) break;
      memmove(rx, rx + used, have - used);
      have -= used;
      if (have == 0 || r == SvParse::NEED_MORE) break;
    }
    if (have >= RX_CAP) have = 0;  // overflow guard: drop and keep listening
  }
  portENTER_CRITICAL(&mux); snap.busErrors++; portEXIT_CRITICAL(&mux);
  setError(sawBadSum ? "reply checksum error" : sawWrongId ? "reply from wrong ID" : "reply timeout");
  return false;
}

// A reply with a non-zero status byte is still a valid reply: the data is returned and
// the status is recorded in snap.replyStatus (the FSM ORs it into its fault check).
bool readRegs(uint8_t addr, uint8_t n, uint8_t* out) {
  const uint8_t q[2] = {addr, n};
  uint8_t st = 0, got = 0;
  if (!transact(SV_READ, q, 2, st, out, got)) return false;
  if (got != n) { setError("reply length mismatch"); return false; }
  return true;
}

// Delivered (true) whenever the servo acknowledged, whatever its status byte says.
bool writeRegs(uint8_t addr, const uint8_t* data, uint8_t n) {
  uint8_t q[16];
  if (n > sizeof(q) - 1) { setError("write too long"); return false; }
  q[0] = addr; memcpy(q + 1, data, n);
  uint8_t st = 0, got = 0;
  return transact(SV_WRITE, q, (uint8_t)(n + 1), st, nullptr, got);
}

void put16(uint16_t v, uint8_t* p) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

bool torqueOff() { const uint8_t off = 0; return writeRegs(40, &off, 1); }

// Stop = torque off only (Waveshare CF library EnableTorque(ID, 0)), retried once.
// Nothing else is written: on this servo writing a goal register (41..47) re-enables
// torque and starts motion (observed: the gripper drove to OPEN while in FAULT after
// a goal-speed write).
bool stopWrites() {
  return torqueOff() || torqueOff();
}

// Cleanup stop after a failed verb: keeps the original error, appends a note if the
// stop also failed.
void cleanupStop() {
  char orig[sizeof(snap.lastError)];
  getError(orig, sizeof(orig));
  if (stopWrites()) { setError(orig); return; }
  char e[sizeof(snap.lastError)];
  snprintf(e, sizeof(e), "%.40s; stop failed too", orig);
  setError(e);
}

// Every motion uses position mode 0 and the firmware never switches mode on its own:
// on this servo a write to reg 33 resets the position count (observed: Open changed the
// mode to 0 and the position jumped to ~0, breaking the saved limits). The mode is only
// read here; "Set position mode" (servoSetPositionMode) is an explicit operator action.
bool requireMode0() {
  bool known; uint8_t cur;
  portENTER_CRITICAL(&mux); known = snap.modeKnown; cur = snap.mode; portEXIT_CRITICAL(&mux);
  if (known && cur == 0) return true;
  setError(known ? "servo not in position mode 0" : "servo mode not read yet");
  return false;
}

uint16_t clampU16(int32_t v, int32_t lo, int32_t hi) { return (uint16_t)(v < lo ? lo : v > hi ? hi : v); }

// Block 41..47 exactly as the Waveshare CF library writes it (WritePosEx / WriteSpe):
// accel u8, goal position SM15, goal torque (max current, 6.5 mA units) u16, goal speed
// SM15 (0.732 RPM units). Writing this block is what starts the motion; the library
// never writes torque enable (reg 40) for a move, and writing it after the goal can
// make the servo take its present position as the goal (Open used to time out without
// moving).
bool writeGoalBlock(int32_t pos, int32_t speed, uint16_t torqueLimit) {
  uint8_t d[7];
  d[0] = ACCEL;
  put16(svEncodeSM15(pos), d + 1);
  put16(clampU16(torqueLimit, 1, 1000), d + 3);
  put16(svEncodeSM15(speed), d + 5);
  return writeRegs(41, d, 7);
}
}  // namespace

void servoBegin(uint8_t id, uint32_t baud) {
  servoId = id;
  portENTER_CRITICAL(&mux);
  memset(&snap, 0, sizeof(snap));
  everGood = false;
  portEXIT_CRITICAL(&mux);
  phaseLatch = {false, 0};
  busBegin(baud);
}

bool servoPoll() {
  uint8_t b[18];
  if (!readRegs(56, 18, b)) {
    portENTER_CRITICAL(&mux); snap.online = false; portEXIT_CRITICAL(&mux);
    return false;
  }
  const ServoFeedback fb = svDecodeFeedback(b);
  portENTER_CRITICAL(&mux);
  snap.fb = fb; snap.rawPosition = fb.position; snap.fb.position = fb.position + posOffset;
  snap.online = true; lastGoodMs = millis(); everGood = true;
  portEXIT_CRITICAL(&mux);
  return true;
}
// servoPoll does not clear lastError: a verb failure stays visible until the next verb succeeds.

bool servoReadModeAndPhase() {
  uint8_t mode = 255, phase = 0;
  static uint8_t modeFails = 0;              // UART-task-only
  if (!readRegs(33, 1, &mode) || mode > 3) {
    svPhaseUpdate(phaseLatch, false, 0);       // counts as a failed phase read
    // Like fullAngle: one dropped reply keeps the last good mode; 3 in a row clear it.
    if (modeFails < 255) modeFails++;
    portENTER_CRITICAL(&mux);
    if (modeFails >= 3) { snap.modeKnown = false; snap.mode = 255; }
    snap.fullAngle = phaseLatch.fullAngle;
    portEXIT_CRITICAL(&mux);
    return false;
  }
  modeFails = 0;
  portENTER_CRITICAL(&mux); snap.mode = mode; snap.modeKnown = true; portEXIT_CRITICAL(&mux);
  const bool phaseOk = readRegs(18, 1, &phase);
  svPhaseUpdate(phaseLatch, phaseOk, phase);   // one dropped reply keeps the last good value
  portENTER_CRITICAL(&mux); snap.fullAngle = phaseLatch.fullAngle; portEXIT_CRITICAL(&mux);
  return phaseOk;
}

bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit) {
  if (speed < 1) speed = 1;
  if (!requireMode0()) return false;          // nothing was written: nothing to undo
  if (writeGoalBlock(position - posOffset, speed, torqueLimit)) { clearError(); return true; }
  cleanupStop();  // best effort; lastError keeps the original failure
  return false;
}

// The gripper cannot be back-driven, so "hold" is torque off: the jaw stays where it is.
bool servoHoldHere(uint16_t, uint16_t) { return servoStop(); }

// Approach / grasp in position mode: drive toward the close limit (or OPEN zero) at the
// given speed; the FSM stops it with servoStop() on contact / force / limit. Torque goes
// off first so a slower goal (contact -> grasp) does not ride the previous speed's ramp.
bool servoRunAt(int16_t speedClosingPositive, uint16_t torqueLimit) {
  if (speedClosingPositive == 0) return servoStop();
  if (!requireMode0()) return false;
  stopWrites();                               // best effort
  const int32_t mag = speedClosingPositive < 0 ? -(int32_t)speedClosingPositive : speedClosingPositive;
  return servoMoveTo(speedClosingPositive > 0 ? closeLimitPos : openZeroPos, (uint16_t)mag, torqueLimit);
}

// Jog step in position mode: goal = present position +/- a chunk that outlasts the
// 300 ms jog dead-man at this speed (re-issued on every repeat), like Release 3.0.3.
// target is clamped by the caller when limits apply.
bool servoJogTo(int32_t target, uint16_t speed, uint16_t torqueLimit) {
  return servoMoveTo(target, speed, torqueLimit);
}

bool servoStop() {
  if (!stopWrites()) return false;
  clearError();
  return true;
}

bool servoBootStop() { return servoStop(); }

// Operator action: torque off, unlock EEPROM, mode 0, lock (as the library's
// ProgramEprom example brackets EEPROM writes). The position count may reset.
bool servoSetPositionMode() {
  if (!stopWrites()) return false;
  const uint8_t unlock = 0, lock = 1, mode0 = 0;
  if (!writeRegs(55, &unlock, 1)) return false;
  const bool ok = writeRegs(33, &mode0, 1);
  writeRegs(55, &lock, 1);            // best effort
  portENTER_CRITICAL(&mux);
  if (ok) { snap.mode = 0; snap.modeKnown = true; } else { snap.modeKnown = false; snap.mode = 255; }
  portEXIT_CRITICAL(&mux);
  if (ok) clearError();
  return ok;
}

// Waveshare CalibrationOfs(): torque off, unlock EEPROM (reg 55 = 0), calibrate
// instruction 0x0B (current position becomes the servo's middle). The EEPROM is locked
// again afterwards (the library leaves it unlocked).
bool servoCalibrateMiddle() {
  if (!stopWrites()) return false;
  const uint8_t unlock = 0, lock = 1;
  if (!writeRegs(55, &unlock, 1)) return false;
  uint8_t st = 0, n = 0;
  const bool ok = transact(SV_CAL, nullptr, 0, st, nullptr, n);
  writeRegs(55, &lock, 1);            // best effort
  if (ok) clearError();
  return ok;
}

bool servoScanBaud(uint32_t& foundBaud) {
  static const uint32_t bauds[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
  const uint32_t original = servoBaud;
  for (uint32_t baud : bauds) {
    busBegin(baud);
    uint8_t st = 0, n = 0;
    if (transact(SV_PING, nullptr, 0, st, nullptr, n)) { foundBaud = baud; clearError(); return true; }
  }
  busBegin(original);
  setError("no servo found at any baud");
  return false;
}

ServoSnapshot servoGet() {
  ServoSnapshot s;
  portENTER_CRITICAL(&mux);
  s = snap;
  s.ageMs = everGood ? (uint32_t)(millis() - lastGoodMs) : UINT32_MAX;
  portEXIT_CRITICAL(&mux);
  return s;
}

void servoSetLimits(int32_t closeLimit, int32_t openZero, bool closePositive) {
  (void)closePositive;   // targets are absolute positions
  closeLimitPos = closeLimit; openZeroPos = openZero;
}

void servoSetOffset(int32_t offset) {
  portENTER_CRITICAL(&mux);
  snap.fb.position += offset - posOffset;      // keep the snapshot consistent until the next poll
  posOffset = offset;
  portEXIT_CRITICAL(&mux);
}
int32_t servoGetOffset() { return posOffset; }
