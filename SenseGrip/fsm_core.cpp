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
  lastPos_ = in.position;
  approachOnly_ = in.approachOnly;
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
      return startUpdate(in);

    case State::PRE_TOUCH:
      if (in.anyFingerContact) return enter(State::GRASP, "finger contact");
      if (preStep_ == PreTouchStep::APPROACH) {
        if (std::abs((long)in.position - (long)progPos_) >= 5) { progPos_ = in.position; progMs_ = nowMs_; }
        const bool there = std::abs((long)in.position - (long)plan_.approachGoal) <= (long)cfg_.openToleranceCounts;
        if (there || (uint32_t)(nowMs_ - progMs_) >= APPROACH_STALL_MS) {
          if (approachOnly_) return enter(State::MONITOR, "approach only: stopped at pre-touch position");
          preStep_ = PreTouchStep::SLOW;
          entryMs_ = nowMs_;                       // the slow step gets its own PRE_TOUCH timeout
          stepTimeoutMs_ = travelTimeoutMs(cfg_.preTouchTimeoutMs, cfg_.contactSpeed);
          setReason("pre-touch position reached: slow to contact");
          return act(MotorAction::RUN, cfg_.contactSpeed, 0, plan_.prof.maxCurrent);
        }
      }
      if (pastCloseLimit(in.position)) return enter(State::FAULT, "no contact before close limit");
      if (timedOut(in, preStep_ == PreTouchStep::SLOW ? stepTimeoutMs_ : cfg_.preTouchTimeoutMs)) return enter(State::FAULT, "PRE_TOUCH timeout");
      return NO_ACTION;

    case State::GRASP:
      if (graspReached(in)) {
        if (tightening_) return tightenEnd(true, "tighten done: higher force reached");
        heldByGrasp_ = true;
        return enter(State::MONITOR, "grasp force reached");
      }
      if (pastCloseLimit(in.position))
        return tightening_ ? tightenEnd(false, "tighten failed: close limit reached") : enter(State::FAULT, "target force not reached before close limit");
      if (timedOut(in, stepTimeoutMs_)) return tightening_ ? tightenEnd(false, "tighten failed: timeout") : enter(State::FAULT, "GRASP timeout");
      return NO_ACTION;

    case State::MONITOR:
      // Automatic tighten only for a real grasp (not an approach-only stop) and while armed.
      if (in.slipEvent && in.tightenEnabled && armed_ && heldByGrasp_) return tryTighten("slip");
      return NO_ACTION;                          // holds until the operator presses Open

    case State::OPEN: {
      const long off = std::abs((long)in.position - (long)cfg_.openZero);
      if (std::abs((long)in.position - (long)progPos_) >= 5) { progPos_ = in.position; progMs_ = nowMs_; }
      const bool settled = (uint32_t)(nowMs_ - progMs_) >= OPEN_SETTLE_MS && off <= OPEN_SETTLE_COUNTS;
      if (off <= (long)cfg_.openToleranceCounts || settled) {
        openedSinceBoot_ = true;
        armed_ = false;
        if (off <= (long)cfg_.openToleranceCounts) return enter(State::IDLE, "at OPEN");
        std::snprintf(scratch_, sizeof scratch_, "at OPEN (servo stopped %ld counts short)", off);
        return enter(State::IDLE, scratch_);
      }
      if (timedOut(in, cfg_.openTimeoutMs)) {
        std::snprintf(scratch_, sizeof scratch_, "OPEN timeout: stopped %ld counts from OPEN zero", off);
        return enter(State::FAULT, scratch_);
      }
      return NO_ACTION;
    }

    case State::FAULT:
      return NO_ACTION;                          // waits for Reset
  }
  return NO_ACTION;
}

// Entry actions. Sets state, reason and entry time; returns the motor action.
MotorAction Fsm::enter(State s, const char* why) {
  State prev = state_;
  state_ = s;
  if (s != State::GRASP && tightening_) { tightening_ = false; tevent_ = TightenEvent::FAILED; }   // aborted (FAULT, Open, ...)
  if (s == State::START || s == State::IDLE) heldByGrasp_ = false;
  entryMs_ = nowMs_;
  setReason("%s", why);
  switch (s) {
    case State::IDLE:
      if (prev == State::OPEN) return act(MotorAction::HOLD, cfg_.openSpeed, 0, cfg_.openTorqueLimit);
      return NO_ACTION;                          // from START: never moved; from FAULT: torque stays off
    case State::START:
      tightenCount_ = 0; tightening_ = false;
      planEverValid_ = false; haveCls_ = false; clsSinceMs_ = nowMs_; startFirst_ = true;
      return NO_ACTION;
    case State::PRE_TOUCH: {
      const int32_t ahead = cfg_.closePositive ? plan_.approachGoal - lastPos_ : lastPos_ - plan_.approachGoal;
      if (plan_.approach && ahead > (int32_t)cfg_.openToleranceCounts) {
        preStep_ = PreTouchStep::APPROACH;
        progPos_ = lastPos_; progMs_ = nowMs_;
        return act(MotorAction::MOVE, cfg_.approachSpeed, plan_.approachGoal, plan_.prof.maxCurrent);
      }
      preStep_ = PreTouchStep::SLOW;
      stepTimeoutMs_ = travelTimeoutMs(cfg_.preTouchTimeoutMs, cfg_.contactSpeed);
      if (approachOnly_) return enter(State::MONITOR, "approach only: no pre-touch position to move to (not measured, or already there)");
      return act(MotorAction::RUN, cfg_.contactSpeed, 0, plan_.prof.maxCurrent);
    }
    case State::GRASP:
      graspCount_ = 0;
      graspSeq_ = fsrSeq_;                       // count only samples newer than the contact one
      stepTimeoutMs_ = travelTimeoutMs(cfg_.graspTimeoutMs, plan_.prof.graspSpeed);
      return act(MotorAction::RUN, plan_.prof.graspSpeed, 0, plan_.prof.maxCurrent);
    case State::MONITOR:
      return act(MotorAction::HOLD, plan_.prof.graspSpeed, 0, plan_.prof.maxCurrent);
    case State::OPEN:
      progPos_ = lastPos_; progMs_ = nowMs_;
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
    case Cmd::TIGHTEN:
      if (state_ != State::MONITOR) { setReason("Tighten ignored: not in MONITOR"); return NO_ACTION; }
      return tryTighten("manual");
  }
  return NO_ACTION;
}

// Disarm while closing (START/PRE_TOUCH/GRASP) releases: behaves like Open.
// Elsewhere it only clears the flag (MONITOR keeps holding until Open).
MotorAction Fsm::onDisarm() {
  armed_ = false;
  if (state_ == State::GRASP && tightening_) return tightenEnd(false, "Disarmed during tighten - holding");
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

// Tighten (phase 4b): re-close at a higher target. Never for the delicate class, at most
// TIGHTEN_MAX per grasp, never above TIGHTEN_CAP x Max force. why: "slip" or "manual".
MotorAction Fsm::tryTighten(const char* why) {
  if (plan_.cls == ObjClass::DELICATE) { setReason("Tighten refused (%s): delicate object", why); return NO_ACTION; }
  if (tightenCount_ >= TIGHTEN_MAX) { setReason("Tighten refused (%s): already tightened %u times", why, (unsigned)TIGHTEN_MAX); return NO_ACTION; }
  const float prev = plan_.prof.forceN, cap = TIGHTEN_CAP * cfg_.maxForceN;
  float next = prev * TIGHTEN_GAIN;
  if (next > cap) next = cap;
  if (!(next > prev + 0.01f)) { setReason("Tighten refused (%s): target %.2f N already at the cap (%.2f N)", why, (double)prev, (double)cap); return NO_ACTION; }
  plan_.prof.forceN = next;
  tightenCount_++; tightening_ = true; tevent_ = TightenEvent::STARTED;
  std::snprintf(scratch_, sizeof scratch_, "%s: tightening %u/%u to %.2f N", why, (unsigned)tightenCount_, (unsigned)TIGHTEN_MAX, (double)next);
  return enter(State::GRASP, scratch_);
}

// Tighten finished: back to MONITOR either way (a failed tighten is not a FAULT; the object is still held).
MotorAction Fsm::tightenEnd(bool ok, const char* why) {
  tightening_ = false;
  tevent_ = ok ? TightenEvent::DONE : TightenEvent::FAILED;
  return enter(State::MONITOR, why);
}

// START: wait until the plan has been valid with one class for START_STABLE_MS, then latch it.
MotorAction Fsm::startUpdate(const FsmInputs& in) {
  // The class seen at the first START update counts from START entry (the step that entered
  // START had the same inputs).
  const bool first = startFirst_;
  startFirst_ = false;
  if (!in.plan.valid) { haveCls_ = false; clsSinceMs_ = nowMs_; }
  else {
    planEverValid_ = true;
    if (!haveCls_ || in.plan.cls != stableCls_) {
      haveCls_ = true; stableCls_ = in.plan.cls; clsSinceMs_ = first ? entryMs_ : nowMs_;
    }
    if ((uint32_t)(nowMs_ - clsSinceMs_) >= START_STABLE_MS) {
      if (!(in.plan.prof.forceN < cfg_.maxForceN)) {
        armed_ = false;
        static const char* const NAMES[4] = {"small", "medium", "large", "delicate"};
        std::snprintf(scratch_, sizeof scratch_, "Start aborted: %s target force %.1f N is not below Max force %.1f N - raise Max force (FSR tab)",
                      NAMES[(int)in.plan.cls & 3], (double)in.plan.prof.forceN, (double)cfg_.maxForceN);
        return enter(State::IDLE, scratch_);
      }
      plan_ = in.plan;
      return enter(State::PRE_TOUCH, plan_.approach ? "approaching pre-touch position" : "closing slowly to contact");
    }
  }
  if ((uint32_t)(nowMs_ - entryMs_) >= START_TIMEOUT_MS) {
    armed_ = false;
    if (!planEverValid_) std::snprintf(scratch_, sizeof scratch_, "Start aborted: %s", in.planWhy ? in.planWhy : "object not measured");
    else std::snprintf(scratch_, sizeof scratch_, "Start aborted: object size not stable");
    return enter(State::IDLE, scratch_);
  }
  return NO_ACTION;
}

// Counts consecutive new FSR samples meeting the grasp target.
bool Fsm::graspReached(const FsmInputs& in) {
  if (in.fsrSeq == graspSeq_) return false;
  graspSeq_ = in.fsrSeq;
  bool ok = in.avgContactForce >= plan_.prof.forceN && in.channelsInContact >= cfg_.minActiveFsr;
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

uint32_t Fsm::travelTimeoutMs(uint32_t baseMs, uint16_t speed) const {
  const long dist = std::abs((long)cfg_.closeLimit - (long)lastPos_);
  const uint32_t rate = (uint32_t)(speed ? speed : 1) * COUNTS_PER_S_PER_SPEED;
  const uint32_t need = (uint32_t)((uint64_t)dist * 1500 / rate) + 2000;
  return need > baseMs ? need : baseMs;
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
