#include <Arduino.h>
#include <freertos/queue.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "control.h"
#include "config.h"
#include "fsr.h"
#include "servo.h"
#include "shared.h"
#include "tof.h"

static QueueHandle_t gQueue = nullptr;
static ControlStatus gStatus;
static volatile bool gEstopReq = false;
static GripConfig gCfgPub;

static constexpr uint32_t JOG_DEADMAN_MS = 300;
static constexpr uint32_t SERVO_FRESH_MS = 100;
static constexpr uint32_t BOOT_STOP_RETRY_MS = 20;
static constexpr uint32_t BOOT_STOP_WINDOW_MS = 1000;

void controlBegin() {
  memset(&gStatus, 0, sizeof(gStatus));
  gQueue = xQueueCreate(8, sizeof(ControlRequest));
}

bool controlEnqueue(const ControlRequest& r) {
  if (!gQueue) return false;
  return xQueueSend(gQueue, &r, 0) == pdTRUE;
}

void controlRequestEstop() {
  portENTER_CRITICAL(&gMux);
  gEstopReq = true;
  portEXIT_CRITICAL(&gMux);
}

ControlStatus controlGet() { return snap(gStatus); }
GripConfig controlGetConfig() { return snap(gCfgPub); }

namespace {

struct Ctl {
  GripConfig cfg;
  Fsm fsm;
  bool jogActive = false;
  int16_t jogSpeed = 0;
  bool stopPending = false;
  uint8_t stopFails = 0;
  uint32_t jogLastMs = 0;
  char msg[96] = "";
  bool msgOk = true;
  bool bootStopPending = false;   // servo not yet confirmed stopped after reset
  explicit Ctl(const GripConfig& c) : cfg(c), fsm(c) {}
};

void say(Ctl& c, bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
void say(Ctl& c, bool ok, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(c.msg, sizeof(c.msg), fmt, ap);
  va_end(ap);
  c.msgOk = ok;
}

void applyConfig(Ctl& c) {
  portENTER_CRITICAL(&gMux);
  gCfgPub = c.cfg;
  portEXIT_CRITICAL(&gMux);
  c.fsm.setConfig(c.cfg);
  servoSetRunViaPosition(c.cfg.runViaPosition, c.cfg.closeLimit, c.cfg.openZero, c.cfg.closePositive);
  tofSetDetect(c.cfg.detectMm, c.cfg.detectHoldMs, c.cfg.tofMask);
}

// Stops a jog; a failed servoStop() is retried every iteration (see retryStop).
void jogStop(Ctl& c) {
  if (!c.jogActive) return;
  c.jogActive = false;
  if (!servoStop()) { c.stopPending = true; c.stopFails = 1; }
}

void retryStop(Ctl& c) {
  if (!c.stopPending) return;
  if (servoStop()) { c.stopPending = false; c.stopFails = 0; return; }
  if (++c.stopFails >= 3) {
    c.stopPending = false;
    c.stopFails = 0;
    c.fsm.motorFailed(servoGet().lastError);
  }
}

// Soft-limit check for a jog; only enforced once limits are valid.
bool jogPastLimit(const GripConfig& g, int16_t speed, int32_t pos) {
  if (!g.limitsValid) return false;
  bool closing = speed > 0;
  if (closing) return g.closePositive ? pos >= g.closeLimit : pos <= g.closeLimit;
  return g.closePositive ? pos <= g.openZero : pos >= g.openZero;
}

void handleRequest(Ctl& c, const ControlRequest& r, uint32_t now, Cmd& cmdOut) {
  if (r.kind == ControlRequest::FSM_CMD) {
    jogStop(c);
    cmdOut = r.cmd;
    return;
  }
  if (r.kind == ControlRequest::JOG_STOP) {
    jogStop(c);
    say(c, true, "jog stopped");
    return;
  }
  const char* what = r.kind == ControlRequest::JOG ? "jog"
                   : r.kind == ControlRequest::SET_OPEN ? "set open"
                   : r.kind == ControlRequest::SET_CLOSE ? "set close"
                   : r.kind == ControlRequest::SCAN ? "scan" : "config";
  State st = c.fsm.state();
  if (st != State::IDLE && st != State::FAULT) {
    say(c, false, "%s only allowed in IDLE or FAULT (now %s)", what, stateName(st));
    return;
  }
  ServoSnapshot sv = servoGet();

  switch (r.kind) {
    case ControlRequest::JOG: {
      if (c.fsm.armed()) { say(c, false, "disarm before jogging"); return; }
      if (c.cfg.runViaPosition && !c.cfg.limitsValid) {
        say(c, false, "set limits before enabling run-via-position jog");
        return;
      }
      if (!sv.online) { say(c, false, "jog refused: servo offline"); return; }
      int16_t sp = r.speed;
      if (sp > 61) sp = 61;
      if (sp < -61) sp = -61;
      if (sp == 0) { jogStop(c); say(c, true, "jog stopped"); return; }
      if (jogPastLimit(c.cfg, sp, sv.fb.position)) {
        jogStop(c);
        say(c, false, "jog refused: %s limit reached", sp > 0 ? "close" : "open");
        return;
      }
      uint16_t tq = sp > 0 ? c.cfg.closeTorqueLimit : c.cfg.openTorqueLimit;
      c.fsm.invalidateOpened();            // the jaw may leave OPEN (even if the command fails)
      if (!servoRunAt(sp, tq)) {
        c.jogActive = true;   // so jogStop() sends and retries the stop
        jogStop(c);
        say(c, false, "jog failed: %s", servoGet().lastError);
        return;
      }
      c.jogActive = true;
      c.jogSpeed = sp;
      c.jogLastMs = now;
      say(c, true, "jogging");
      return;
    }
    case ControlRequest::SET_OPEN:
    case ControlRequest::SET_CLOSE: {
      bool isOpen = r.kind == ControlRequest::SET_OPEN;
      if (!sv.online || sv.ageMs > SERVO_FRESH_MS) { say(c, false, "%s refused: no fresh servo position", what); return; }
      jogStop(c);
      GripConfig n = c.cfg;
      if (isOpen) n.openZero = sv.fb.position; else n.closeLimit = sv.fb.position;
      n.limitsValid = true;
      const char* prob = limitsProblem(n);
      n.limitsValid = (prob == nullptr);
      if (!configSave(n)) { say(c, false, "%s failed: could not save config", what); return; }
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();            // OPEN must be reached again against the new limits
      if (prob) say(c, true, "%s saved at %ld; limits not valid yet (%s)", what, (long)sv.fb.position,
                    isOpen ? "set CLOSE limit on the closing side" : "set OPEN zero on the opening side");
      else say(c, true, "%s saved at %ld", what, (long)sv.fb.position);
      return;
    }
    case ControlRequest::SCAN: {
      jogStop(c);
      uint32_t found = 0;
      if (servoScanBaud(found)) {
        uint8_t id; uint32_t baud;
        servoIdBaudLoad(id, baud);
        servoIdBaudSave(id, found);
        servoBegin(id, found);
        say(c, true, "servo found at %lu baud", (unsigned long)found);
      } else {
        say(c, false, "scan: no servo found");
      }
      return;
    }
    case ControlRequest::CONFIG: {
      GripConfig n = r.cfg;
      // Limits are owned by SET_OPEN/SET_CLOSE; the request may carry a stale snapshot.
      n.openZero = c.cfg.openZero; n.closeLimit = c.cfg.closeLimit; n.limitsValid = c.cfg.limitsValid;
      clampConfig(n);
      if (n.limitsValid) {
        const char* prob = limitsProblem(n);
        if (prob) { say(c, false, "config refused: %s", prob); return; }
      }
      if (!configSave(n)) { say(c, false, "config failed: could not save"); return; }
      const bool motionChanged = n.closePositive != c.cfg.closePositive ||
                                 n.runViaPosition != c.cfg.runViaPosition;
      c.cfg = n;
      applyConfig(c);
      if (motionChanged) c.fsm.invalidateOpened();   // press Open again with the new direction/mode
      say(c, true, "config saved");
      return;
    }
    default: return;
  }
}

FsmInputs buildInputs(uint32_t now, Cmd cmd) {
  FsmInputs in;
  memset(&in, 0, sizeof(in));
  ServoSnapshot sv = servoGet();
  FsrSnapshot fs = fsrGet();
  TofSnapshot tf = tofGet();
  in.nowMs = now; in.cmd = cmd;
  // Fault bits come from the status register and from the status byte of each reply.
  in.servoOnline = sv.online; in.servoAgeMs = sv.ageMs;
  in.servoStatus = (uint8_t)(sv.fb.status | sv.replyStatus);
  in.servoFullAngle = sv.fullAngle; in.servoModeKnown = sv.modeKnown;
  in.position = sv.fb.position;
  bool any = false;
  for (int i = 0; i < 6; i++) { in.fsrCalibrated[i] = fs.calibrated[i]; in.fsrForce[i] = fs.force[i]; }
  for (int f = 0; f < 3; f++) any = any || fs.contacts.stable[f];
  in.anyFingerContact = any;
  in.avgContactForce = fs.agg.avgContactForce;
  in.channelsInContact = fs.agg.channelsInContact;
  in.maxForce = fs.agg.maxForce;
  in.fsrSeq = fs.seq;
  in.tofOnline = tf.online; in.objectPresent = tf.objectPresent;
  return in;
}

bool applyAction(const MotorAction& a) {
  switch (a.kind) {
    case MotorAction::RUN:  return servoRunAt(a.speed, a.torque);
    case MotorAction::HOLD: return servoHoldHere(a.torque, a.speed);
    case MotorAction::MOVE: return servoMoveTo(a.position, a.speed, a.torque);
    case MotorAction::STOP: return servoStop();
    default: return true;
  }
}

void publish(Ctl& c) {
  ControlStatus s;
  memset(&s, 0, sizeof(s));
  s.state = c.fsm.state(); s.faultFrom = c.fsm.faultFrom();
  s.armed = c.fsm.armed(); s.openedSinceBoot = c.fsm.openedSinceBoot();
  snprintf(s.reason, sizeof(s.reason), "%s", c.fsm.reason());
  snprintf(s.faultReason, sizeof(s.faultReason), "%s", c.fsm.faultReason());
  s.nBlockers = c.fsm.blockers(s.blockers, Fsm::MAX_REPORTED);
  snprintf(s.lastActionMsg, sizeof(s.lastActionMsg), "%s", c.msg);
  s.lastActionOk = c.msgOk;
  portENTER_CRITICAL(&gMux);
  memcpy(&gStatus, &s, sizeof(s));
  portEXIT_CRITICAL(&gMux);
}

// After an ESP32 reset the servo keeps its last mode, torque and goal speed (it may
// still be closing). Stop it before the control loop starts: retry every 20 ms for
// up to 1 s; if it still fails, the loop keeps retrying every iteration.
void bootStop(Ctl& c) {
  const uint32_t t0 = millis();
  for (;;) {
    if (servoBootStop()) { say(c, true, "servo stopped at boot (torque off, goal speed 0)"); return; }
    if ((uint32_t)(millis() - t0) >= BOOT_STOP_WINDOW_MS) break;
    vTaskDelay(pdMS_TO_TICKS(BOOT_STOP_RETRY_MS));
  }
  c.bootStopPending = true;
  say(c, false, "could not stop servo at boot: %s", servoGet().lastError);
}

void retryBootStop(Ctl& c) {
  if (!c.bootStopPending) return;
  // Once the FSM or a jog drives the servo, its commands replace the stale goal.
  State st = c.fsm.state();
  if (c.jogActive || (st != State::IDLE && st != State::FAULT)) { c.bootStopPending = false; return; }
  if (servoBootStop()) {
    c.bootStopPending = false;
    say(c, true, "servo stopped (after boot retries)");
  } else {
    say(c, false, "could not stop servo at boot: %s", servoGet().lastError);
  }
}

}  // namespace

void controlTask(void*) {
  Ctl* c = new Ctl(configLoad());
  applyConfig(*c);                     // no motion at boot
  bootStop(*c);
  // Mode/phase are read on the first iteration so Open is not refused for the first second.
  uint32_t lastServo = 0, lastFsr = 0, lastMode = 0;
  bool modeRead = false;
  for (;;) {
    uint32_t now = millis();
    Cmd cmd = Cmd::NONE;

    bool estop;
    portENTER_CRITICAL(&gMux);
    estop = gEstopReq;
    gEstopReq = false;
    portEXIT_CRITICAL(&gMux);

    if (estop) {
      jogStop(*c);
      cmd = Cmd::ESTOP;
    } else if (gQueue) {
      ControlRequest r;
      if (xQueueReceive(gQueue, &r, 0) == pdTRUE) handleRequest(*c, r, now, cmd);
    }

    // The FSM owns the motor whenever it is not IDLE/FAULT: drop the jog without touching the servo.
    State fst = c->fsm.state();
    if (c->jogActive && fst != State::IDLE && fst != State::FAULT) c->jogActive = false;
    retryStop(*c);
    retryBootStop(*c);
    if (c->jogActive && c->cfg.limitsValid) {
      ServoSnapshot js = servoGet();
      if (!js.online || js.ageMs > SERVO_FRESH_MS) {
        jogStop(*c);
        say(*c, false, "jog stopped: stale servo position");
      } else if (jogPastLimit(c->cfg, c->jogSpeed, js.fb.position)) {
        jogStop(*c);
        say(*c, false, "jog stopped: %s limit reached", c->jogSpeed > 0 ? "close" : "open");
      }
    }
    if (c->jogActive && (uint32_t)(now - c->jogLastMs) >= JOG_DEADMAN_MS) {
      jogStop(*c);
      say(*c, true, "jog stopped (no repeat within %u ms)", (unsigned)JOG_DEADMAN_MS);
    }

    if ((uint32_t)(now - lastServo) >= 10) { lastServo = now; servoPoll(); }
    if ((uint32_t)(now - lastFsr) >= 20) { lastFsr = now; fsrSample(c->cfg.contactForceN, c->cfg.contactSamples); }
    if (!modeRead || (uint32_t)(now - lastMode) >= 1000) { modeRead = true; lastMode = now; servoReadModeAndPhase(); }

    MotorAction a = c->fsm.step(buildInputs(now, cmd));
    if (a.kind != MotorAction::NONE && !applyAction(a)) {
      c->fsm.motorFailed(servoGet().lastError);
      servoStop();
    }

    publish(*c);
    vTaskDelay(1);
  }
}
