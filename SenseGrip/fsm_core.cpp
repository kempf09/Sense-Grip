#include "fsm_core.h"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include "pins.h"
#include "servo_proto.h"

static constexpr uint32_t SERVO_STALE_MS = 100;
static constexpr uint8_t SERVO_FAULT_BITS = 0x2F;
static constexpr uint32_t STOP_RETRY_MS = 100;

static MotorAction act(MotorAction::Kind k, int16_t speed = 0, int32_t pos = 0, uint16_t torque = 0) {
  return MotorAction{k, speed, pos, torque};
}
static const MotorAction NO_ACTION = act(MotorAction::NONE);

static bool servoOffline(const FsmInputs& in) { return !in.servoOnline || in.servoAgeMs > SERVO_STALE_MS; }

// States in which the servo may be moving or holding a grip.
static bool moving(State s) { return s != State::IDLE && s != State::FAULT; }

const char* stateName(State s) {
  switch (s) {
    case State::IDLE:      return "IDLE";
    case State::START:     return "START";
    case State::PRE_TOUCH: return "PRE_TOUCH";
    case State::GRASP:     return "GRASP";
    case State::MONITOR:   return "MONITOR";
    case State::OPEN:      return "OPEN";
    case State::FAULT:     return "FAULT";
  }
  return "?";
}

Fsm::Fsm(const GripConfig& c) : cfg_(c) { setReason("power-up"); }

void Fsm::setConfig(const GripConfig& c) { cfg_ = c; }

// ---------------------------------------------------------------------------
// The state machine: global safety checks -> operator command -> per-state.
// ---------------------------------------------------------------------------
MotorAction Fsm::step(const FsmInputs& in) {
  nowMs_ = in.nowMs;
  fsrSeq_ = in.fsrSeq;
  computeBlockers(in);
  MotorAction a = NO_ACTION;

  if (const char* trip = moving(state_) ? safetyTrip(in) : nullptr) {
    a = enter(State::FAULT, trip);
  } else {
    State before = state_;
    a = handleCommand(in);
    if (a.kind == MotorAction::NONE && state_ == before) a = run(in);
  }

  if (stopPending_ && a.kind == MotorAction::NONE && (int32_t)(nowMs_ - stopDueMs_) >= 0) {
    stopPending_ = false;                       // set again if this STOP fails
    a = act(MotorAction::STOP);
  }
  return a;
}

MotorAction Fsm::run(const FsmInputs& in) {
  switch (state_) {
    case State::IDLE:
      if (armed_ && in.objectPresent) return enter(State::START, "object detected");
      return NO_ACTION;

    case State::START:
      if (!armed_) return enter(State::IDLE, "Start aborted: not armed");
      if (nBlockers_ > 0) {
        armed_ = false;
        std::snprintf(scratch_, sizeof scratch_, "Start aborted: %s", blockers_[0].text);
        return enter(State::IDLE, scratch_);
      }
      return enter(State::PRE_TOUCH, "approaching object");

    case State::PRE_TOUCH:
      if (in.anyFingerContact) return enter(State::GRASP, "finger contact");
      if (pastCloseLimit(in.position)) return enter(State::FAULT, "no contact before close limit");
      if (timedOut(in, cfg_.preTouchTimeoutMs)) return enter(State::FAULT, "PRE_TOUCH timeout");
      return NO_ACTION;

    case State::GRASP:
      if (graspReached(in)) return enter(State::MONITOR, "grasp force reached");
      if (pastCloseLimit(in.position)) return enter(State::FAULT, "target force not reached before close limit");
      if (timedOut(in, cfg_.graspTimeoutMs)) return enter(State::FAULT, "GRASP timeout");
      return NO_ACTION;

    case State::MONITOR:
      return NO_ACTION;                          // holds until the operator presses Open

    case State::OPEN:
      if (std::abs((long)in.position - (long)cfg_.openZero) <= (long)cfg_.openToleranceCounts) {
        openedSinceBoot_ = true;
        armed_ = false;
        return enter(State::IDLE, "at OPEN");
      }
      if (timedOut(in, cfg_.openTimeoutMs)) return enter(State::FAULT, "OPEN timeout");
      return NO_ACTION;

    case State::FAULT:
      return NO_ACTION;                          // waits for Reset
  }
  return NO_ACTION;
}

// Entry actions. Sets state, reason and entry time; returns the motor action.
MotorAction Fsm::enter(State s, const char* why) {
  State prev = state_;
  state_ = s;
  entryMs_ = nowMs_;
  setReason("%s", why);
  switch (s) {
    case State::IDLE:
      if (prev == State::OPEN) return act(MotorAction::HOLD, cfg_.openSpeed, 0, cfg_.openTorqueLimit);
      return NO_ACTION;                          // from START: never moved; from FAULT: torque stays off
    case State::START:
      return NO_ACTION;
    case State::PRE_TOUCH:
      return act(MotorAction::RUN, cfg_.approachSpeed, 0, cfg_.closeTorqueLimit);
    case State::GRASP:
      graspCount_ = 0;
      graspSeq_ = fsrSeq_;                       // count only samples newer than the contact one
      return act(MotorAction::RUN, cfg_.graspSpeed, 0, cfg_.closeTorqueLimit);
    case State::MONITOR:
      return act(MotorAction::HOLD, cfg_.graspSpeed, 0, cfg_.closeTorqueLimit);
    case State::OPEN:
      return act(MotorAction::MOVE, cfg_.openSpeed, cfg_.openZero, cfg_.openTorqueLimit);
    case State::FAULT:
      if (prev != State::FAULT) {               // keep the original cause on re-entry
        faultFrom_ = prev;
        std::snprintf(faultReason_, sizeof faultReason_, "%s", why);
      }
      armed_ = false;
      openedSinceBoot_ = false;                  // torque off: operator must press Open again
      return act(MotorAction::STOP);
  }
  return NO_ACTION;
}

void Fsm::motorFailed(const char* err) {
  bool wasFault = state_ == State::FAULT;
  std::snprintf(scratch_, sizeof scratch_, "servo command failed: %s", err ? err : "?");
  enter(State::FAULT, scratch_);
  // STOP on the next step(); if we were already in FAULT (a STOP just failed), retry in 100 ms.
  stopPending_ = true;
  stopDueMs_ = wasFault ? nowMs_ + STOP_RETRY_MS : nowMs_;
}

// ---------------------------------------------------------------------------
// Operator commands
// ---------------------------------------------------------------------------
MotorAction Fsm::handleCommand(const FsmInputs& in) {
  switch (in.cmd) {
    case Cmd::NONE:   return NO_ACTION;
    case Cmd::ESTOP:  return enter(State::FAULT, "E-stop");
    case Cmd::DISARM: return onDisarm();
    case Cmd::ARM:    onArm(); return NO_ACTION;
    case Cmd::OPEN:   return onOpen(in);
    case Cmd::RESET:  return onReset(in);
  }
  return NO_ACTION;
}

// Disarm while closing (START/PRE_TOUCH/GRASP) releases: behaves like Open.
// Elsewhere it only clears the flag (MONITOR keeps holding until Open).
MotorAction Fsm::onDisarm() {
  armed_ = false;
  if (state_ == State::START || state_ == State::PRE_TOUCH || state_ == State::GRASP) {
    std::snprintf(scratch_, sizeof scratch_, "Disarmed during %s — opening", stateName(state_));
    return enter(State::OPEN, scratch_);
  }
  setReason("disarmed");
  return NO_ACTION;
}

void Fsm::onArm() {
  if (state_ != State::IDLE) setReason("Arm ignored: not in IDLE");
  else if (nBlockers_ > 0) setReason("Arm refused: %s", blockers_[0].text);
  else { armed_ = true; setReason("armed"); }
}

MotorAction Fsm::onOpen(const FsmInputs& in) {
  if (state_ == State::OPEN) return NO_ACTION;
  if (state_ == State::FAULT) { setReason("Open refused: Reset first"); return NO_ACTION; }
  if (servoOffline(in)) { setReason("Open refused: servo offline or feedback stale"); return NO_ACTION; }
  if (in.servoStatus & SERVO_FAULT_BITS) {
    char bits[40];
    svStatusText(in.servoStatus, bits, sizeof bits);
    setReason("Open refused: servo status: %s", bits);
    return NO_ACTION;
  }
  if (!in.servoModeKnown || in.servoMode != 0) { setReason("Open refused: servo not in position mode 0"); return NO_ACTION; }
  if (!in.servoFullAngle) { setReason("Open refused: servo multi-turn (full-angle) feedback off"); return NO_ACTION; }
  if (!in.homed) { setReason("Open refused: not homed — press Home"); return NO_ACTION; }
  if (const char* p = limitsProblem(cfg_)) { setReason("Open refused: %s", p); return NO_ACTION; }
  return enter(State::OPEN, "Open requested");
}

MotorAction Fsm::onReset(const FsmInputs& in) {
  if (state_ != State::FAULT) { setReason("Reset ignored: not in FAULT"); return NO_ACTION; }
  if (servoOffline(in)) { setReason("Reset refused: servo offline or feedback stale"); return NO_ACTION; }
  if (in.servoStatus & SERVO_FAULT_BITS) {
    char bits[40];
    svStatusText(in.servoStatus, bits, sizeof bits);
    setReason("Reset refused: servo status: %s", bits);
    return NO_ACTION;
  }
  armed_ = false;
  return enter(State::IDLE, "reset");
}

// ---------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------
// Global safety checks while moving; returns the FAULT reason or nullptr.
const char* Fsm::safetyTrip(const FsmInputs& in) {
  if (in.maxForce >= cfg_.maxForceN) return "FSR force limit";
  if (servoOffline(in)) return "servo offline or feedback stale";
  if (in.servoStatus & SERVO_FAULT_BITS) {
    char bits[40];
    svStatusText(in.servoStatus, bits, sizeof bits);
    std::snprintf(scratch_, sizeof scratch_, "servo status: %s", bits);
    return scratch_;
  }
  if (outsideTravel(in.position)) return "position outside travel limits";
  return nullptr;
}

// Counts consecutive new FSR samples meeting the grasp target.
bool Fsm::graspReached(const FsmInputs& in) {
  if (in.fsrSeq == graspSeq_) return false;
  graspSeq_ = in.fsrSeq;
  bool ok = in.avgContactForce >= cfg_.graspForceN && in.channelsInContact >= cfg_.minActiveFsr;
  graspCount_ = ok ? graspCount_ + 1 : 0;
  return graspCount_ >= cfg_.graspSamples;
}

// "Reached" includes the last openToleranceCounts: in position mode the servo stops at
// (or just short of) its goal, which is the close limit itself.
bool Fsm::pastCloseLimit(int32_t pos) const {
  const int32_t tol = cfg_.openToleranceCounts;
  return cfg_.closePositive ? pos >= cfg_.closeLimit - tol : pos <= cfg_.closeLimit + tol;
}

bool Fsm::outsideTravel(int32_t pos) const {
  int32_t slack = cfg_.limitSlackCounts;
  if (cfg_.closePositive) return pos < cfg_.openZero - slack || pos > cfg_.closeLimit + slack;
  return pos > cfg_.openZero + slack || pos < cfg_.closeLimit - slack;
}

bool Fsm::timedOut(const FsmInputs& in, uint32_t limitMs) const {
  return (uint32_t)(in.nowMs - entryMs_) >= limitMs;   // wrap-safe
}

// ---------------------------------------------------------------------------
// Blockers ("why won't it go")
// ---------------------------------------------------------------------------
void Fsm::computeBlockers(const FsmInputs& in) {
  nBlockers_ = 0;
  if (servoOffline(in)) addBlocker(BLK_SERVO_OFFLINE, "servo offline or feedback stale");
  if (in.servoStatus & SERVO_FAULT_BITS) {
    char bits[40];
    svStatusText(in.servoStatus, bits, sizeof bits);
    addBlocker(BLK_SERVO_STATUS, "servo status: %s", bits);
  }
  if (in.servoModeKnown && in.servoMode != 0)
    addBlocker(BLK_SERVO_MODE, "servo in mode %u, not position mode 0 — Servo tab: Set position mode", (unsigned)in.servoMode);
  if (!in.servoFullAngle) addBlocker(BLK_FULL_ANGLE, "servo multi-turn (full-angle) feedback off");
  if (const char* p = limitsProblem(cfg_)) addBlocker(BLK_LIMITS, "%s", p);
  if (!in.homed) addBlocker(BLK_NOT_HOMED, "not homed — press Home (gripper must be empty)");
  if (!openedSinceBoot_) addBlocker(BLK_NOT_OPENED, "not at OPEN — press Open (after power-up, jog, new limits)");
  for (int i = 0; i < FSR_COUNT; i++) {
    if (!in.fsrCalibrated[i]) addBlocker(BLK_FSR_UNCALIBRATED, "FSR %d not calibrated", FSR_CHANNEL_NUMBER[i]);
    else if (state_ == State::IDLE && in.fsrForce[i] >= cfg_.contactForceN)
      addBlocker(BLK_FSR_LOADED, "FSR %d loaded at rest", FSR_CHANNEL_NUMBER[i]);
  }
  if (in.tofOnline == 0) addBlocker(BLK_NO_TOF, "no ToF sensor online");
}

void Fsm::addBlocker(uint8_t id, const char* fmt, ...) {
  if (nBlockers_ >= MAX_BLOCKERS) return;
  Blocker& b = blockers_[nBlockers_++];
  b.id = id;
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(b.text, sizeof b.text, fmt, ap);   // truncates at 63 chars
  va_end(ap);
}

uint8_t Fsm::blockers(Blocker* out, uint8_t cap) const {
  uint8_t n = 0;
  for (; n < nBlockers_ && n < cap; n++) out[n] = blockers_[n];
  if (!armed_ && n < cap) {
    out[n].id = BLK_NOT_ARMED;
    std::snprintf(out[n].text, sizeof out[n].text, "not armed");
    n++;
  }
  return n;
}

void Fsm::setReason(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(reason_, sizeof reason_, fmt, ap);
  va_end(ap);
}
