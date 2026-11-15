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
  bool jogIgnoreLimits = false;
  bool stopPending = false;
  uint8_t stopFails = 0;
  uint32_t jogLastMs = 0;
  char msg[96] = "";
  bool msgOk = true;
  bool bootStopPending = false;   // servo not yet confirmed stopped after reset
  uint32_t loopMaxUs = 0, loopMaxWin = 0, loopWinStart = 0;
  uint32_t posJumps = 0;
  bool havePos = false; int32_t lastPos = 0; uint32_t lastPosMs = 0;
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
  servoSetLimits(c.cfg.closeLimit, c.cfg.openZero, c.cfg.closePositive);
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
                   : r.kind == ControlRequest::SCAN ? "scan"
                   : r.kind == ControlRequest::CLEAR_LIMITS ? "clear limits"
                   : r.kind == ControlRequest::CALIBRATE_MIDDLE ? "calibrate middle"
                   : r.kind == ControlRequest::SET_POSITION_MODE ? "set position mode" : "config";
  State st = c.fsm.state();
  if (st != State::IDLE && st != State::FAULT) {
    say(c, false, "%s only allowed in IDLE or FAULT (now %s)", what, stateName(st));
    return;
  }
  ServoSnapshot sv = servoGet();

  switch (r.kind) {
    case ControlRequest::JOG: {
      if (c.fsm.armed()) { say(c, false, "disarm before jogging"); return; }
      if (!sv.online) { say(c, false, "jog refused: servo offline"); return; }
      int16_t sp = r.speed;
      if (sp > (int16_t)SPEED_MAX) sp = SPEED_MAX;
      if (sp < -(int16_t)SPEED_MAX) sp = -(int16_t)SPEED_MAX;
      if (sp == 0) { jogStop(c); say(c, true, "jog stopped"); return; }
      if (!r.ignoreLimits && jogPastLimit(c.cfg, sp, sv.fb.position)) {
        jogStop(c);
        say(c, false, "jog refused: %s limit reached (tick 'Ignore limits' to go past it)", sp > 0 ? "close" : "open");
        return;
      }
      uint16_t tq = sp > 0 ? c.cfg.closeTorqueLimit : c.cfg.openTorqueLimit;
      // Position-mode jog step: a goal ahead of the present position that outlasts the
      // dead-man at this speed (speed unit 0.732 RPM ~ 50 counts/s), re-issued on repeat.
      const int32_t mag = sp < 0 ? -(int32_t)sp : sp;
      const int32_t dir = ((sp > 0) == c.cfg.closePositive) ? 1 : -1;
      int32_t target = sv.fb.position + dir * mag * 30;      // ~0.6 s of travel
      if (c.cfg.limitsValid && !r.ignoreLimits) {
        const int32_t lim = sp > 0 ? c.cfg.closeLimit : c.cfg.openZero;
        if ((dir > 0 && target > lim) || (dir < 0 && target < lim)) target = lim;
      }
      if (target > 32767) target = 32767;
      if (target < -32767) target = -32767;
      c.fsm.invalidateOpened();            // the jaw may leave OPEN (even if the command fails)
      if (!servoJogTo(target, (uint16_t)mag, tq)) {
        c.jogActive = true;   // so jogStop() sends and retries the stop
        jogStop(c);
        say(c, false, "jog failed: %s", servoGet().lastError);
        return;
      }
      c.jogActive = true;
      c.jogSpeed = sp;
      c.jogIgnoreLimits = r.ignoreLimits;
      c.jogLastMs = now;
      say(c, true, r.ignoreLimits ? "jogging (limits ignored)" : "jogging");
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
    case ControlRequest::CLEAR_LIMITS: {
      jogStop(c);
      GripConfig n = c.cfg;
      n.limitsValid = false;
      if (!configSave(n)) { say(c, false, "clear limits failed: could not save config"); return; }
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();
      say(c, true, "limits cleared: jog freely, then set OPEN zero and close limit again");
      return;
    }
    case ControlRequest::SET_POSITION_MODE: {
      jogStop(c);
      if (c.fsm.armed()) { say(c, false, "disarm first"); return; }
      if (!sv.online) { say(c, false, "set position mode refused: servo offline"); return; }
      if (!servoSetPositionMode()) { say(c, false, "set position mode failed: %s", servoGet().lastError); return; }
      // A mode write can reset the position count: the saved limits no longer apply.
      GripConfig n = c.cfg;
      n.limitsValid = false;
      configSave(n);
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();
      say(c, true, "servo in position mode 0 (saved); limits cleared - jog, set OPEN zero and close limit again");
      return;
    }
    case ControlRequest::CALIBRATE_MIDDLE: {
      jogStop(c);
      if (c.fsm.armed()) { say(c, false, "disarm before calibrating"); return; }
      if (!sv.online) { say(c, false, "calibrate middle refused: servo offline"); return; }
      if (!servoCalibrateMiddle()) { say(c, false, "calibrate middle failed: %s", servoGet().lastError); return; }
      // Every position the servo reports has shifted: saved limits no longer match the jaw.
      GripConfig n = c.cfg;
      n.limitsValid = false;
      configSave(n);
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();
      say(c, true, "servo middle calibrated here (torque off); limits cleared - set OPEN zero and close limit again");
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
      const bool motionChanged = n.closePositive != c.cfg.closePositive;
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
  in.servoFullAngle = sv.fullAngle; in.servoModeKnown = sv.modeKnown; in.servoMode = sv.mode;
  in.position = sv.fb.position;
  bool any = false;
  // A channel the operator disabled is ignored by the FSM: not required to be calibrated,
  // never "loaded at rest", never counted toward the force limit.
  const uint8_t en = controlGetConfig().fsrEnabled;
  for (int i = 0; i < 6; i++) {
    const bool used = en & (1u << i);
    in.fsrCalibrated[i] = fs.calibrated[i] || !used;
    in.fsrForce[i] = used ? fs.force[i] : 0.0f;
  }
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

// Cause-independent guard: the jaw cannot move faster than ~3100 counts/s (speed 61), so a
// position change of more than 400 counts within 50 ms means the servo's position count
// was reset (e.g. by a mode change). The saved limits and "opened" no longer apply.
void checkPositionJump(Ctl& c, uint32_t now) {
  ServoSnapshot sv = servoGet();
  if (!sv.online || sv.ageMs > SERVO_FRESH_MS) { c.havePos = false; return; }
  const int32_t pos = sv.fb.position;
  if (c.havePos && (uint32_t)(now - c.lastPosMs) <= 50) {
    const int32_t d = pos - c.lastPos;
    if (d > 400 || d < -400) {
      c.posJumps++;
      State st = c.fsm.state();
      if (st != State::IDLE && st != State::FAULT) c.fsm.motorFailed("servo position jumped (count reset?)");
      jogStop(c);
      if (c.cfg.limitsValid) {
        GripConfig n = c.cfg; n.limitsValid = false; configSave(n); c.cfg = n; applyConfig(c);
      }
      c.fsm.invalidateOpened();
      say(c, false, "servo position jumped %ld -> %ld: limits cleared, set them again", (long)c.lastPos, (long)pos);
    }
  }
  c.havePos = true; c.lastPos = pos; c.lastPosMs = now;
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
  s.loopMaxUs = c.loopMaxUs; s.posJumps = c.posJumps;
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
    if (servoBootStop()) { say(c, true, "servo stopped at boot (torque off)"); return; }
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
  // Rate-limited: with the servo unreachable each attempt costs bus timeouts.
  static uint32_t lastTry = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - lastTry) < 250) return;
  lastTry = now;
  if (servoBootStop()) {
    c.bootStopPending = false;
    say(c, true, "servo stopped (after boot retries)");
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
    const uint32_t loopStartUs = micros();
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
    if (c->jogActive && c->cfg.limitsValid && !c->jogIgnoreLimits) {
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

    if ((uint32_t)(now - lastServo) >= 10) { lastServo = now; if (servoPoll()) checkPositionJump(*c, now); }
    if ((uint32_t)(now - lastFsr) >= 20) { lastFsr = now; fsrSample(c->cfg.contactForceN, c->cfg.contactSamples, c->cfg.fsrEnabled); }
    if (!modeRead || (uint32_t)(now - lastMode) >= 1000) { modeRead = true; lastMode = now; servoReadModeAndPhase(); }

    MotorAction a = c->fsm.step(buildInputs(now, cmd));
    if (a.kind != MotorAction::NONE && !applyAction(a)) {
      c->fsm.motorFailed(servoGet().lastError);
      servoStop();
    }

    {   // loop timing diagnostics: max iteration time over 5 s windows
      const uint32_t dt = micros() - loopStartUs;
      if (dt > c->loopMaxWin) c->loopMaxWin = dt;
      if ((uint32_t)(now - c->loopWinStart) >= 5000) { c->loopMaxUs = c->loopMaxWin; c->loopMaxWin = 0; c->loopWinStart = now; }
    }
    publish(*c);
    vTaskDelay(1);
  }
}
