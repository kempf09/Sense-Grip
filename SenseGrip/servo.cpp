#include "servo.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include "pins.h"

namespace {
HardwareSerial Bus(1);
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint32_t REPLY_TIMEOUT_US = 25000;
constexpr uint8_t  ACCEL = 20;
constexpr size_t   RX_CAP = 64;

uint8_t  servoId = 1;
uint32_t servoBaud = 1000000;

// Shared state: only touched inside mux (except UART-task-only config below).
ServoSnapshot snap = {};
uint32_t lastGoodMs = 0;
bool     everGood = false;

// UART-task-only.
bool runViaPosition = false;
int32_t closeLimitPos = 0, openZeroPos = 0;
bool closePositiveDir = true;

void setError(const char* text) {
  portENTER_CRITICAL(&mux);
  strncpy(snap.lastError, text, sizeof(snap.lastError) - 1);
  snap.lastError[sizeof(snap.lastError) - 1] = '\0';
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
    if (!got) { taskYIELD(); continue; }
    for (;;) {
      const uint8_t* d = nullptr; uint8_t dn = 0, st = 0; size_t used = 0;
      SvParse r = svParseReply(rx, have, servoId, st, d, dn, used);
      if (r == SvParse::OK) {
        status = st; outN = dn;
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
  setError(sawBadSum ? "reply checksum error" : sawWrongId ? "reply from wrong ID" : "reply timeout");
  return false;
}

bool readRegs(uint8_t addr, uint8_t n, uint8_t* out, uint8_t* statusOut = nullptr) {
  const uint8_t q[2] = {addr, n};
  uint8_t st = 0, got = 0;
  if (!transact(SV_READ, q, 2, st, out, got)) return false;
  if (got != n) { setError("reply length mismatch"); return false; }
  if (statusOut) *statusOut = st;
  else if (st != 0) {
    char e[48]; snprintf(e, sizeof(e), "servo status 0x%02X", st); setError(e); return false;
  }
  return true;
}

bool writeRegs(uint8_t addr, const uint8_t* data, uint8_t n) {
  uint8_t q[16];
  if (n > sizeof(q) - 1) { setError("write too long"); return false; }
  q[0] = addr; memcpy(q + 1, data, n);
  uint8_t st = 0, got = 0;
  if (!transact(SV_WRITE, q, (uint8_t)(n + 1), st, nullptr, got)) return false;
  if (st != 0) {
    char e[48]; snprintf(e, sizeof(e), "servo status 0x%02X", st); setError(e); return false;
  }
  return true;
}

void put16(uint16_t v, uint8_t* p) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

bool torqueOff() { const uint8_t off = 0; return writeRegs(40, &off, 1); }

bool ensureMode(uint8_t want) {
  bool known; uint8_t cur;
  portENTER_CRITICAL(&mux); known = snap.modeKnown; cur = snap.mode; portEXIT_CRITICAL(&mux);
  if (known && cur == want) return true;
  torqueOff();  // best effort: never switch mode with torque on (stale goal would jump)
  if (!writeRegs(33, &want, 1)) {
    // The write may have taken effect even though the reply failed.
    portENTER_CRITICAL(&mux); snap.modeKnown = false; snap.mode = 255; portEXIT_CRITICAL(&mux);
    return false;
  }
  portENTER_CRITICAL(&mux); snap.mode = want; snap.modeKnown = true; portEXIT_CRITICAL(&mux);
  return true;
}

bool torqueOn() { const uint8_t one = 1; return writeRegs(40, &one, 1); }

uint16_t clampU16(int32_t v, int32_t lo, int32_t hi) { return (uint16_t)(v < lo ? lo : v > hi ? hi : v); }

// Block 41: accel u8, goal pos SM15, torque limit u16, goal speed u16.
bool writePositionBlock(int32_t pos, uint16_t speed, uint16_t torqueLimit) {
  uint8_t d[7];
  d[0] = ACCEL;
  put16(svEncodeSM15(pos), d + 1);
  put16(clampU16(torqueLimit, 1, 430), d + 3);
  put16(clampU16(speed, 1, 32767), d + 5);
  return writeRegs(41, d, 7);
}
}  // namespace

void servoBegin(uint8_t id, uint32_t baud) {
  servoId = id;
  portENTER_CRITICAL(&mux);
  memset(&snap, 0, sizeof(snap));
  everGood = false;
  portEXIT_CRITICAL(&mux);
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
  snap.fb = fb; snap.online = true; lastGoodMs = millis(); everGood = true;
  portEXIT_CRITICAL(&mux);
  return true;
}

bool servoReadModeAndPhase() {
  uint8_t mode = 255, phase = 0;
  if (!readRegs(33, 1, &mode) || mode > 3) {
    portENTER_CRITICAL(&mux); snap.modeKnown = false; snap.mode = 255; portEXIT_CRITICAL(&mux);
    return false;
  }
  portENTER_CRITICAL(&mux); snap.mode = mode; snap.modeKnown = true; portEXIT_CRITICAL(&mux);
  const bool phaseOk = readRegs(18, 1, &phase);
  portENTER_CRITICAL(&mux); snap.fullAngle = phaseOk && (phase & 0x10) != 0; portEXIT_CRITICAL(&mux);
  return phaseOk;
}

bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit) {
  // Goal first, torque last, so torque never comes on with a stale goal.
  if (ensureMode(0) && writePositionBlock(position, speed, torqueLimit) && torqueOn()) return true;
  servoStop();  // best effort cleanup; lastError keeps the original failure unless stop also fails
  return false;
}

bool servoHoldHere(uint16_t torqueLimit, uint16_t speed) {
  bool online, fresh; int32_t pos;
  portENTER_CRITICAL(&mux);
  online = snap.online; pos = snap.fb.position;
  fresh = everGood && (uint32_t)(millis() - lastGoodMs) <= 100;
  portEXIT_CRITICAL(&mux);
  if (!online) { setError("hold: servo offline"); return false; }
  if (!fresh) { setError("stale feedback"); return false; }
  return servoMoveTo(pos, speed, torqueLimit);
}

bool servoRunAt(int16_t speedClosingPositive, uint16_t torqueLimit) {
  if (speedClosingPositive == 0) return servoHoldHere(torqueLimit, 1);
  if (runViaPosition) {
    const int32_t mag = speedClosingPositive < 0 ? -(int32_t)speedClosingPositive : speedClosingPositive;
    return servoMoveTo(speedClosingPositive > 0 ? closeLimitPos : openZeroPos,
                       (uint16_t)mag, torqueLimit);
  }
  if (!ensureMode(1)) return false;
  const int32_t signedSpeed = closePositiveDir ? speedClosingPositive : -(int32_t)speedClosingPositive;
  uint8_t d[4];  // regs 44..45 torque limit, 46..47 goal speed
  put16(clampU16(torqueLimit, 1, 430), d);
  put16(svEncodeSM15(signedSpeed), d + 2);
  // Speed and torque limit first, torque last.
  if (writeRegs(44, d, 4) && torqueOn()) return true;
  servoStop();
  return false;
}

bool servoStop() {
  const uint8_t off = 0;
  return writeRegs(40, &off, 1);
}

bool servoScanBaud(uint32_t& foundBaud) {
  static const uint32_t bauds[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
  const uint32_t original = servoBaud;
  for (uint32_t baud : bauds) {
    busBegin(baud);
    uint8_t st = 0, n = 0;
    if (transact(SV_PING, nullptr, 0, st, nullptr, n)) { foundBaud = baud; return true; }
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

void servoSetRunViaPosition(bool v, int32_t closeLimit, int32_t openZero, bool closePositive) {
  runViaPosition = v; closeLimitPos = closeLimit; openZeroPos = openZero; closePositiveDir = closePositive;
}
