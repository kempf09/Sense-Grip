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
#include "homing_core.h"
#include "pins.h"
#include "sweep_store.h"
#include "object_core.h"
#include "plan_core.h"
#include "contact_core.h"
#include "log_core.h"
#include <climits>

static QueueHandle_t gQueue = nullptr;
static ControlStatus gStatus;
static volatile bool gEstopReq = false;
static GripConfig gCfgPub;
static LogRing gLog = {nullptr, 0, 0};
const LogRing* controlLog() { return gLog.rows ? &gLog : nullptr; }

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
  uint32_t fsrUsMax = 0, fsrUsMaxWin = 0;
  bool havePos = false; int32_t lastPos = 0; uint32_t lastPosMs = 0;
  // Homing (close-to-touch). homed: the logical position frame is valid since this boot.
  bool homed = false;
  bool homing = false;
  uint32_t homeT0 = 0, homeStepMs = 0, homeWinMs = 0, homeUnstickT0 = 0;
  int32_t homeWinPos = 0;
  bool homeUnstickUsed = false, homeUnstickActive = false;
  uint8_t homeContact = 0; uint32_t homeFsrSeq = 0;
  int32_t homeResidual = 0;
  char homeMsg[96] = "not homed since power-up";
  bool pendingOpen = false;
  bool openZeroSetThisBoot = false;
  // Calibration sweep (spec §6.3)
  bool sweeping = false; uint8_t swPhase = 0; uint32_t swT0 = 0, swPhaseMs = 0;
  int swN = 0; int32_t swTarget = 0, swStepStart = 0, swProgPos = 0; uint32_t swProgMs = 0; uint32_t swOk[3] = {0, 0, 0}; uint8_t swContact = 0; uint32_t swFsrSeq = 0;
  SensorCalib cal[3];
  // SEN0628 power-up estimate and running position check (spec §6.4)
  PosEstimate est = {false, 0, 0}; int32_t estRaw = 0; uint32_t estMs = 0;
  PosEstimate estAtHome = {false, 0, 0}; int32_t estRawAtHome = 0;
  uint32_t checkMs = 0, mismatchSinceMs = 0; bool mismatching = false;
  // Phase 3: object perception, grasp plan, contact pattern (spec §7)
  ObjectSmoother sm = {}; uint32_t objOk[3] = {0, 0, 0};
  Override ov = Override::AUTO; bool delicate = false, approachOnly = false;   // RAM only
  PlanResult pr = {};
  FirstContact first[3] = {FirstContact::NONE, FirstContact::NONE, FirstContact::NONE};
  State lastState = State::IDLE;
  // Phase 4a log: events waiting for a row (one event per row)
  State logState = State::IDLE; FirstContact logFirst[3] = {FirstContact::NONE, FirstContact::NONE, FirstContact::NONE};
  bool pendGrasp = false; uint8_t pendContact = 0;   // bit f: finger f's first contact not yet logged
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

// ---- Homing (close-to-touch), spec §6.2 ----
// Home and sweep close with the configured close current (closeTorqueLimit, default 200 = 1.3 A).
// 120 (0.8 A) was too weak: on hardware the sweep stalled after 20-40 steps as the mechanism stiffens.
constexpr uint16_t HOME_SPEED = 5, UNSTICK_CURRENT = 415;
constexpr uint32_t HOME_TIMEOUT_MS = 120000, UNSTICK_MS = 500, HOME_STEP_MS = 200;
constexpr int32_t HOME_MAX_RESIDUAL = 600;
const uint8_t DISTAL_FSR[3] = {5, 3, 4};               // FINGER_FSR_MAP[f][0]

// Latest SEN0628 frame as sweep-core input.
uint8_t currentFrame(uint16_t frame[3][64]) {
  const TofSnapshot t = tofGet();
  uint8_t valid = 0;
  for (int s = 0; s < 3; s++) {
    memcpy(frame[s], t.s[s].mm, sizeof(frame[s]));
    if (t.s[s].online) valid |= 1u << s;
  }
  return valid;
}

// Fingers visible: >= 6 finger zones read valid now. Without a sweep we cannot tell: report
// visible, so the maximum-current push is never used blindly.
bool fingersVisibleNow(Ctl&) {
  if (!sweepLoaded()) return true;
  uint16_t frame[3][64];
  const uint8_t valid = currentFrame(frame);
  if (valid != 0x7) return true;                         // a sensor offline: cannot tell, deny the push
  const SweepModel& m = sweepModel();
  int zones = 0;
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) if ((m.fingerZones[s] >> z) & 1) zones++;
  if (zones < 6) return true;                            // model too thin to tell
  int n = 0;
  for (int s = 0; s < 3; s++) {
    if (!((valid >> s) & 1)) continue;
    for (int z = 0; z < 64; z++)
      if (((m.fingerZones[s] >> z) & 1) && frame[s][z] > 0 && frame[s][z] < 4000) n++;
  }
  return n >= 6;
}

// The estimate taken when Home started points to a different turn than the homed frame implies.
bool estimateDisagrees(Ctl& c, int32_t posOffset) {
  if (!c.estAtHome.valid) return false;
  return estimateConflicts(c.estAtHome.pos, c.estRawAtHome + posOffset);
}

bool distalTouching(const Ctl& c, const FsrSnapshot& fs) {
  for (uint8_t i : DISTAL_FSR)
    if ((c.cfg.fsrEnabled & (1u << i)) && fs.force[i] == fs.force[i] && fs.force[i] >= c.cfg.contactForceN) return true;
  return false;
}

void homeStop(Ctl& c, bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
void homeStop(Ctl& c, bool ok, const char* fmt, ...) {
  servoStop();
  c.homing = false;
  va_list ap; va_start(ap, fmt);
  vsnprintf(c.homeMsg, sizeof(c.homeMsg), fmt, ap);
  va_end(ap);
  say(c, ok, "Home: %s", c.homeMsg);
}

void homeStep(Ctl& c, const ServoSnapshot& sv, uint16_t current) {
  const int32_t dir = c.cfg.closePositive ? 1 : -1;
  if (!servoJogTo(sv.fb.position + dir * HOME_SPEED * 30, HOME_SPEED, current)) {
    homeStop(c, false, "servo command failed: %s", servoGet().lastError);
  }
}

void homeFinish(Ctl& c) {
  const ServoSnapshot sv = servoGet();
  const HomeResult r = homeResolve(sv.rawPosition, c.cfg.touchRef, HOME_MAX_RESIDUAL);
  c.homeResidual = r.residual;
  if (!r.ok) { homeStop(c, false, "failed: residual %ld counts (limit %ld) - object in the gripper?", (long)r.residual, (long)HOME_MAX_RESIDUAL); return; }
  if (estimateDisagrees(c, r.posOffset)) { homeStop(c, false, "failed: SEN0628 estimate is more than half a turn away"); return; }
  servoStop();
  servoSetOffset(r.posOffset);
  c.havePos = false;                                     // new frame: new jump-guard baseline
  c.homed = true;
  c.homing = false;
  // Limits are kept in the touch-reference frame: a lost turn count cleared them, Home restores them.
  bool restored = false;
  if (!c.cfg.limitsValid && limitsRestorable(c.cfg)) {
    GripConfig n = c.cfg; n.limitsValid = true;
    if (configSave(n)) { c.cfg = n; applyConfig(c); restored = true; }
  }
  snprintf(c.homeMsg, sizeof(c.homeMsg), "homed, residual %ld counts%s", (long)r.residual,
           restored ? ", limits restored" : "");
  say(c, true, "Home: %s - opening", c.homeMsg);
  c.pendingOpen = true;
}

void homeStart(Ctl& c, const ServoSnapshot& sv, uint32_t now) {
  if (c.fsm.armed()) { say(c, false, "Home: disarm first"); return; }
  if (!sv.online || !sv.modeKnown || sv.mode != 0) { say(c, false, "Home refused: servo offline or not in position mode 0"); return; }
  if (!c.cfg.touchValid) { say(c, false, "Home refused: no touch reference yet - run the calibration sweep first"); return; }
  c.estAtHome = c.est; c.estRawAtHome = c.estRaw;       // snapshot before anything moves
  c.homed = false;
  const FsrSnapshot fs = fsrGet();
  bool proximal = false;
  for (uint8_t f = 0; f < 3; f++) {
    const uint8_t i = FINGER_FSR_MAP[f][1];
    if ((c.cfg.fsrEnabled & (1u << i)) && fs.force[i] == fs.force[i] && fs.force[i] >= c.cfg.contactForceN) proximal = true;
  }
  if (proximal) { say(c, false, "Home refused: a base (proximal) pad shows contact - empty the gripper"); return; }
  if (distalTouching(c, fs)) { homeFinish(c); return; }    // fingertips already touching: resolve without moving
  if (fs.agg.channelsInContact > 0) { say(c, false, "Home refused: an FSR shows contact - empty the gripper"); return; }
  c.homing = true; c.homeT0 = now; c.homeStepMs = now; c.homeWinMs = now; c.homeWinPos = sv.fb.position;
  c.homeUnstickUsed = false; c.homeUnstickActive = false; c.homeContact = 0; c.homeFsrSeq = fs.seq;
  snprintf(c.homeMsg, sizeof(c.homeMsg), "homing...");
  say(c, true, "Home: closing slowly to fingertip contact");
  homeStep(c, sv, c.cfg.closeTorqueLimit);
}

// Runs every loop iteration while homing.
void homeUpdate(Ctl& c, uint32_t now) {
  if (!c.homing) return;
  const ServoSnapshot sv = servoGet();
  if (!sv.online || sv.ageMs > SERVO_FRESH_MS) { homeStop(c, false, "aborted: servo feedback lost"); return; }
  if ((uint32_t)(now - c.homeT0) >= HOME_TIMEOUT_MS) { homeStop(c, false, "aborted: no fingertip contact within 120 s"); return; }
  const FsrSnapshot fs = fsrGet();
  if (fs.seq != c.homeFsrSeq) {                          // count consecutive new FSR samples
    c.homeFsrSeq = fs.seq;
    c.homeContact = distalTouching(c, fs) ? (uint8_t)(c.homeContact + 1) : 0;
    if (c.homeContact >= 3) { homeFinish(c); return; }
  }
  if (c.homeUnstickActive && (uint32_t)(now - c.homeUnstickT0) >= UNSTICK_MS) c.homeUnstickActive = false;
  // Stall check over 300 ms windows.
  const int32_t moved = sv.fb.position - c.homeWinPos;
  if (moved >= 10 || moved <= -10) { c.homeWinPos = sv.fb.position; c.homeWinMs = now; }
  else if (stalled(sv.fb.position, c.homeWinPos, now - c.homeWinMs)) {
    const bool contact = fs.agg.channelsInContact > 0;
    if (!c.homeUnstickUsed && unstickAllowed(c.homed, sv.fb.position, c.cfg.openZero, c.cfg.closePositive,
                                             fingersVisibleNow(c), contact, now - c.homeT0)) {
      c.homeUnstickUsed = true; c.homeUnstickActive = true; c.homeUnstickT0 = now;
      c.homeWinMs = now;
      say(c, true, "Home: jaw stuck past open - one push at maximum current");
      homeStep(c, sv, UNSTICK_CURRENT);
      return;
    }
    homeStop(c, false, "failed: jaw stopped at %ld without tip contact (%d mA, load %.0f%%)",
             (long)sv.fb.position, (int)(sv.fb.current * 6.5f), (double)(sv.fb.load * 100.0f));
    return;
  }
  if ((uint32_t)(now - c.homeStepMs) >= HOME_STEP_MS) {
    c.homeStepMs = now;
    homeStep(c, sv, c.homeUnstickActive ? UNSTICK_CURRENT : c.cfg.closeTorqueLimit);
  }
}

// ---- Calibration sweep (spec §6.3) ----
// Step, speed and current come from the config (sweepStepCounts / sweepSpeed / sweepCurrent).
uint16_t sweepCurrentOf(const GripConfig& g) { return g.sweepCurrent ? g.sweepCurrent : g.closeTorqueLimit; }
constexpr uint32_t SWEEP_TIMEOUT_MS = 400000, SWEEP_FRAME_WAIT_MS = 1000, SWEEP_MOVE_WAIT_MS = 5000;
enum : uint8_t { SW_TO_OPEN = 0, SW_MOVE, SW_FRAMES };

void sweepStop(Ctl& c, bool ok, const char* why) {
  servoStop();
  c.sweeping = false;
  say(c, ok, "Sweep: %s", why);
}

void sweepRecord(Ctl& c, const ServoSnapshot& sv) {
  // Torque off under load can let the jaw spring back a few counts: a sample that is not at least
  // 10 counts past the previous one is skipped (the samples must increase toward closing).
  if (c.swN > 0) {
    const int32_t prev = sweepScratch()[c.swN - 1].pos;
    const int32_t adv = c.cfg.closePositive ? sv.fb.position - prev : prev - sv.fb.position;
    if (adv < 10) return;
  }
  SweepSample& s = sweepScratch()[c.swN];               // never the loaded sweep (abort keeps it intact)
  s.pos = sv.fb.position;
  s.valid = currentFrame(s.mm);
  c.swN++;
}

void sweepFinish(Ctl& c) {
  servoStop();
  c.sweeping = false;
  const ServoSnapshot sv = servoGet();
  const int32_t touch = sv.fb.position;
  char why[80];
  if (const char* bad = sweepInvalidWhy(sweepScratch(), c.swN, c.cfg.openZero, c.cfg.closePositive, why, sizeof why)) {
    say(c, false, "Sweep: not saved - %s", bad);
    return;
  }
  if (const char* err = sweepStoreCommit(c.swN, c.cal, c.cfg.openZero, touch, c.cfg.closePositive)) {
    say(c, false, "Sweep: not saved - %s", err); return;
  }
  GripConfig n = c.cfg;
  n.touchRef = touch; n.touchValid = true;
  n.closeLimit = touch - (n.closePositive ? 1 : -1) * 100;
  n.limitsValid = true;
  if (limitsProblem(n)) n.limitsValid = false;
  if (!configSave(n)) say(c, false, "Sweep: saved the sweep but could not save the touch reference");
  c.cfg = n;
  applyConfig(c);
  c.homed = true;                                        // the sweep defines the logical frame
  snprintf(c.homeMsg, sizeof(c.homeMsg), "homed by calibration sweep");
  say(c, true, "Sweep: saved %d samples, touch at %ld - opening", c.swN, (long)touch);
  c.pendingOpen = true;
}

void sweepStart(Ctl& c, const ServoSnapshot& sv, uint32_t now) {
  if (c.fsm.armed()) { say(c, false, "Sweep: disarm first"); return; }
  if (!sweepScratch()) { say(c, false, "Sweep refused: no memory for the sweep buffer"); return; }
  if (!sv.online || !sv.modeKnown || sv.mode != 0) { say(c, false, "Sweep refused: servo offline or not in position mode 0"); return; }
  // OPEN zero must be valid in the current frame: homed this boot, or set this boot (first-time
  // setup, or re-calibration after Home cannot succeed). A limit from another boot may be turns off.
  if (!c.homed && !c.openZeroSetThisBoot) { say(c, false, "Sweep refused: press Home, or jog to fully open and Set OPEN zero first"); return; }
  if (fsrGet().agg.channelsInContact > 0) { say(c, false, "Sweep refused: an FSR shows contact - empty the gripper"); return; }
  c.sweeping = true; c.swPhase = SW_TO_OPEN; c.swT0 = now; c.swPhaseMs = now; c.swN = 0; c.swContact = 0;
  c.swFsrSeq = fsrGet().seq;
  if (!servoMoveTo(c.cfg.openZero, c.cfg.openSpeed, c.cfg.openTorqueLimit)) { sweepStop(c, false, "servo command failed"); return; }
  say(c, true, "Sweep: moving to OPEN zero");
}

void sweepUpdate(Ctl& c, uint32_t now) {
  if (!c.sweeping) return;
  const ServoSnapshot sv = servoGet();
  if (!sv.online || sv.ageMs > SERVO_FRESH_MS) { sweepStop(c, false, "aborted: servo feedback lost"); return; }
  if ((uint32_t)(now - c.swT0) >= SWEEP_TIMEOUT_MS) { sweepStop(c, false, "aborted: timeout"); return; }
  if (c.swN >= SWEEP_MAX) { sweepStop(c, false, "aborted: too many samples (touch not reached)"); return; }
  const FsrSnapshot fs = fsrGet();
  if (c.swPhase == SW_TO_OPEN && fs.agg.channelsInContact > 0) { sweepStop(c, false, "aborted: FSR contact while moving to OPEN"); return; }
  if (c.swPhase != SW_TO_OPEN && fs.seq != c.swFsrSeq) {
    c.swFsrSeq = fs.seq;
    c.swContact = distalTouching(c, fs) ? (uint8_t)(c.swContact + 1) : 0;
    if (c.swContact >= 3) { sweepFinish(c); return; }
  }
  const int32_t dir = c.cfg.closePositive ? 1 : -1;
  switch (c.swPhase) {
    case SW_TO_OPEN: {
      const int32_t err = sv.fb.position - c.cfg.openZero;
      // The servo can stop a little short of OPEN zero (seen 11-20 counts): stopped within
      // OPEN_SETTLE_COUNTS is close enough (the first sample only has to be within 150).
      if (!sv.fb.moving && err <= OPEN_SETTLE_COUNTS && err >= -OPEN_SETTLE_COUNTS) {
        servoStop();
        c.swPhase = SW_FRAMES; c.swPhaseMs = now;
        const TofSnapshot t = tofGet();
        for (int s = 0; s < 3; s++) c.swOk[s] = t.s[s].ok;
        say(c, true, "Sweep: recording (step %u counts, speed %u, current %u)", (unsigned)c.cfg.sweepStepCounts, (unsigned)c.cfg.sweepSpeed, (unsigned)sweepCurrentOf(c.cfg));
      } else if ((uint32_t)(now - c.swPhaseMs) >= 30000) sweepStop(c, false, "aborted: OPEN zero not reached");
      return;
    }
    case SW_FRAMES: {
      const TofSnapshot t = tofGet();
      bool fresh = true;
      for (int s = 0; s < 3; s++) if (t.s[s].online && t.s[s].ok < c.swOk[s] + 2) fresh = false;
      if (!fresh && (uint32_t)(now - c.swPhaseMs) < SWEEP_FRAME_WAIT_MS) return;
      sweepRecord(c, sv);
      c.swTarget = sv.fb.position + dir * (int32_t)c.cfg.sweepStepCounts;
      if (!servoMoveTo(c.swTarget, c.cfg.sweepSpeed, sweepCurrentOf(c.cfg))) { sweepStop(c, false, "servo command failed"); return; }
      c.swStepStart = sv.fb.position; c.swProgPos = sv.fb.position; c.swProgMs = now;
      c.swPhase = SW_MOVE; c.swPhaseMs = now;
      return;
    }
    case SW_MOVE: {
      const int32_t err = sv.fb.position - c.swTarget;
      bool arrived = !sv.fb.moving && err <= 15 && err >= -15;
      if (sv.fb.position - c.swProgPos >= 5 || sv.fb.position - c.swProgPos <= -5) { c.swProgPos = sv.fb.position; c.swProgMs = now; }
      if (!arrived && (uint32_t)(now - c.swProgMs) >= 500) {
        // No progress for 0.5 s. If the jaw got part of the way, record it where it stopped (the
        // samples only need increasing positions); if it did not move at all, stop and say why.
        const int32_t moved = sv.fb.position - c.swStepStart;
        const int32_t part = (int32_t)c.cfg.sweepStepCounts / 3;   // a third of a step counts as a step
        if (moved >= part || moved <= -part) arrived = true;
        else if ((uint32_t)(now - c.swPhaseMs) >= SWEEP_MOVE_WAIT_MS) {
          char why[96];
          snprintf(why, sizeof why, "stopped at %ld, %ld short, %d mA, load %.0f%%, no tip contact",
                   (long)sv.fb.position, (long)(err < 0 ? -err : err), (int)(sv.fb.current * 6.5f), (double)(sv.fb.load * 100.0f));
          sweepStop(c, false, why);
          return;
        }
      }
      if (!arrived) return;
      servoStop();
      c.swPhase = SW_FRAMES; c.swPhaseMs = now;
      const TofSnapshot t = tofGet();
      for (int s = 0; s < 3; s++) c.swOk[s] = t.s[s].ok;
      return;
    }
  }
}

// ---- SEN0628 power-up estimate and running check (spec §6.4) ----
void estimateUpdate(Ctl& c, uint32_t now) {
  if (!sweepLoaded() || c.homed || c.homing || c.sweeping) return;
  if ((uint32_t)(now - c.estMs) < 500) return;
  c.estMs = now;
  uint16_t frame[3][64];
  const uint8_t valid = currentFrame(frame);
  c.est = estimatePosition(sweepModel(), sweepSamples(), frame, valid);
  c.estRaw = servoGet().rawPosition;
}

void checkUpdate(Ctl& c, uint32_t now) {
  if (!sweepLoaded() || !c.homed || c.homing || c.sweeping) { c.mismatching = false; return; }
  if ((uint32_t)(now - c.checkMs) < 200) return;
  c.checkMs = now;
  const State st = c.fsm.state();
  const ServoSnapshot sv = servoGet();
  const TofSnapshot t = tofGet();
  const int32_t fromOpen = c.cfg.closePositive ? sv.fb.position - c.cfg.openZero : c.cfg.openZero - sv.fb.position;
  if ((st != State::IDLE && st != State::MONITOR) || fromOpen <= FINGERS_INVISIBLE_COUNTS || t.objectPresent || c.jogActive) {
    c.mismatching = false; return;
  }
  uint16_t frame[3][64];
  const uint8_t valid = currentFrame(frame);
  if (viewObstructed(sweepModel(), sweepSamples(), sv.fb.position, frame, valid) ||
      !positionMismatch(sweepModel(), sweepSamples(), sv.fb.position, frame, valid, 25.0f)) { c.mismatching = false; return; }
  if (!c.mismatching) { c.mismatching = true; c.mismatchSinceMs = now; return; }
  if ((uint32_t)(now - c.mismatchSinceMs) > 1000) {
    c.mismatching = false;
    c.homed = false;
    c.fsm.invalidateOpened();
    snprintf(c.homeMsg, sizeof(c.homeMsg), "position check failed - press Home");
    say(c, false, "SEN0628 view disagrees with the motor position: position check failed - press Home");
  }
}

void captureBackground(Ctl& c);   // phase 3, defined below

void handleRequest(Ctl& c, const ControlRequest& r, uint32_t now, Cmd& cmdOut) {
  // Any request (other than another Home press) aborts a running Home.
  if (c.homing && r.kind != ControlRequest::HOME) homeStop(c, false, "aborted by operator request");
  if (c.sweeping && r.kind != ControlRequest::SWEEP) sweepStop(c, false, "aborted by operator request");
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
                   : r.kind == ControlRequest::SET_POSITION_MODE ? "set position mode"
                   : r.kind == ControlRequest::HOME ? "home"
                   : r.kind == ControlRequest::SWEEP ? "sweep" : "config";
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
      // Saved limits live in the touch-reference frame. Not homed, the frame may be whole turns
      // off: a limit set now belongs to a new frame, so the old touch reference no longer applies
      // (re-calibration path: set the limits, then run the calibration sweep).
      const bool dropTouch = n.touchValid && !c.homed;
      if (dropTouch && !r.newFrame) {
        say(c, false, "%s refused: not homed - press Home first (or confirm a new frame, which needs a new calibration sweep)", what);
        return;
      }
      if (dropTouch) n.touchValid = false;
      if (isOpen) n.openZero = sv.fb.position; else n.closeLimit = sv.fb.position;
      if (isOpen) c.openZeroSetThisBoot = true;
      n.limitsValid = true;
      const char* prob = limitsProblem(n);
      n.limitsValid = (prob == nullptr);
      if (!configSave(n)) { say(c, false, "%s failed: could not save config", what); return; }
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();            // OPEN must be reached again against the new limits
      if (prob) say(c, true, "%s saved at %ld; limits not valid yet (%s)", what, (long)sv.fb.position,
                    isOpen ? "set CLOSE limit on the closing side" : "set OPEN zero on the opening side");
      else if (dropTouch) say(c, true, "%s saved at %ld; not homed, so the touch reference was dropped - run the calibration sweep (press Home first next time)", what, (long)sv.fb.position);
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
    case ControlRequest::SWEEP: {
      if (c.sweeping || c.homing) return;
      jogStop(c);
      sweepStart(c, sv, now);
      return;
    }
    case ControlRequest::CAPTURE_BACKGROUND: captureBackground(c); return;
    case ControlRequest::EDIT_OPENING: {
      if (c.sweeping) { say(c, false, "curve edit refused: sweep running"); return; }
      if (const char* err = openingEditSet(r.index, r.mm)) { say(c, false, "curve edit: %s", err); return; }
      if (r.index < 0) say(c, true, "curve edits cleared: analysed curve in use");
      else if (r.mm != r.mm) say(c, true, "curve point %d: edit removed", (int)r.index);
      else say(c, true, "curve point %d set to %.1f mm", (int)r.index, (double)r.mm);
      return;
    }
    case ControlRequest::SET_OVERRIDE: {
      c.ov = r.value <= 3 ? (Override)r.value : Override::AUTO;
      static const char* const N[4] = {"auto", "small", "medium", "large"};
      say(c, true, "class override: %s (until restart)", N[(int)c.ov]);
      return;
    }
    case ControlRequest::SET_DELICATE:
      c.delicate = r.value != 0;
      say(c, true, "delicate mode %s (until restart)", c.delicate ? "ON" : "off");
      return;
    case ControlRequest::SET_APPROACH_ONLY:
      c.approachOnly = r.value != 0;
      say(c, true, "approach only %s (until restart)", c.approachOnly ? "ON: grasps stop at the pre-touch position" : "off");
      return;
    case ControlRequest::HOME: {
      if (c.homing || c.sweeping) return;                 // already running
      jogStop(c);
      homeStart(c, sv, now);
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
      c.havePos = false;                   // the count may reset: take a new baseline
      c.homed = false;
      say(c, true, "servo in position mode 0 (saved); limits cleared - jog, set OPEN zero and close limit again");
      return;
    }
    case ControlRequest::CALIBRATE_MIDDLE: {
      jogStop(c);
      if (c.fsm.armed()) { say(c, false, "disarm before calibrating"); return; }
      if (!sv.online) { say(c, false, "calibrate middle refused: servo offline"); return; }
      if (!servoCalibrateMiddle()) { say(c, false, "calibrate middle failed: %s", servoGet().lastError); return; }
      // Every position the servo reports has shifted: saved limits and the touch reference no
      // longer match the jaw (a new calibration sweep is needed).
      GripConfig n = c.cfg;
      n.limitsValid = false;
      n.touchValid = false;
      c.homed = false;
      configSave(n);
      c.cfg = n;
      applyConfig(c);
      c.fsm.invalidateOpened();
      c.havePos = false;                   // positions shift: take a new baseline
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
      if (const char* prob = profileProblem(n)) { say(c, false, "config refused: %s", prob); return; }
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
  in.maxForce = fs.limitForce;   // max(raw, filtered): filtering never delays the force limit
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

// Cause-independent guard against a lost position count. The jaw cannot be back-driven,
// so with no command running (IDLE/FAULT, no jog) it cannot move at all: any change over
// 400 counts, across any gap (e.g. a servo power glitch), means the count was reset. While
// moving, allow 400 counts plus the maximum rate (~3200 counts/s at speed 61) x elapsed.
void checkPositionJump(Ctl& c, uint32_t now) {
  ServoSnapshot sv = servoGet();
  if (!sv.online || sv.ageMs > SERVO_FRESH_MS) return;      // keep the last known baseline
  const int32_t pos = sv.fb.position;
  if (c.havePos) {
    const int32_t d = pos - c.lastPos;
    const State st = c.fsm.state();
    const bool commanded = c.jogActive || c.homing || c.sweeping || (st != State::IDLE && st != State::FAULT);
    const uint32_t dt = (uint32_t)(now - c.lastPosMs);
    const int64_t allowed = commanded ? 400 + (int64_t)dt * 32 / 10 : 400;
    if (d > allowed || d < -allowed) {
      c.posJumps++;
      if (st != State::IDLE && st != State::FAULT) c.fsm.motorFailed("servo position jumped (count reset?)");
      jogStop(c);
      if (c.cfg.limitsValid) {
        GripConfig n = c.cfg; n.limitsValid = false; configSave(n); c.cfg = n; applyConfig(c);
      }
      c.fsm.invalidateOpened();
      c.homed = false;
      if (c.homing) homeStop(c, false, "aborted: position jumped");
      if (c.sweeping) sweepStop(c, false, "aborted: position jumped");
      say(c, false, "servo position jumped %ld -> %ld: limits cleared, set them again", (long)c.lastPos, (long)pos);
    }
  }
  c.havePos = true; c.lastPos = pos; c.lastPosMs = now;
}

// ---- Phase 3: object measurement, grasp plan, contact pattern (spec §7) ----
// Measures on every new SEN0628 frame set; needs the homed frame and a sweep (background and
// finger zones come from the sweep at the present position).
void objectUpdate(Ctl& c, uint32_t now) {
  if (!c.homed || !sweepLoaded() || c.sweeping || c.homing) { smootherReset(c.sm); return; }
  const TofSnapshot t = tofGet();
  bool fresh = false;
  for (int s = 0; s < 3; s++) if (t.s[s].ok != c.objOk[s]) { fresh = true; c.objOk[s] = t.s[s].ok; }
  if (!fresh) return;
  static uint16_t frame[3][64], bg[3][64];
  uint8_t valid = 0;
  for (int s = 0; s < 3; s++) {
    memcpy(frame[s], t.s[s].mm, sizeof(frame[s]));
    if (t.s[s].online) valid |= 1u << s;
  }
  backgroundAt(sweepModel(), sweepSamples(), servoGet().fb.position, bgDelta(), bg);
  smootherAdd(c.sm, measureObject(frame, valid, bg, sweepModel().fingerZones, c.cal), now);
}

void planUpdate(Ctl& c, uint32_t now) {
  const bool sw = sweepLoaded();
  c.pr = buildPlan(c.cfg, c.homed, sw ? &sweepModel() : nullptr, sw ? sweepSamples() : nullptr,
                   smoothed(c.sm, now), c.ov, c.delicate, servoGet().fb.position);
}

// First pad per finger, from START to MONITOR (reset when a grasp starts).
void contactUpdate(Ctl& c) {
  const State st = c.fsm.state();
  if (st == State::START && c.lastState != State::START)
    for (auto& f : c.first) f = FirstContact::NONE;
  c.lastState = st;
  if (st != State::PRE_TOUCH && st != State::GRASP) return;
  const FsrSnapshot fs = fsrGet();
  bool prox[3], dist[3];
  for (int f = 0; f < 3; f++) {
    const int d = FINGER_FSR_MAP[f][0], p = FINGER_FSR_MAP[f][1];
    dist[f] = (c.cfg.fsrEnabled & (1u << d)) && fs.force[d] == fs.force[d] && fs.force[d] >= c.cfg.contactForceN;
    prox[f] = (c.cfg.fsrEnabled & (1u << p)) && fs.force[p] == fs.force[p] && fs.force[p] >= c.cfg.contactForceN;
  }
  updateFirstContact(c.first, prox, dist);
}

// Capture background: the present empty-gripper view minus the sweep at this position, per
// non-finger zone (re-mounting / new filming location without a full sweep).
void captureBackground(Ctl& c) {
  if (c.fsm.state() != State::IDLE) { say(c, false, "Capture background refused: not in IDLE"); return; }
  if (!c.homed) { say(c, false, "Capture background refused: not homed"); return; }
  if (!sweepLoaded()) { say(c, false, "Capture background refused: no sweep stored"); return; }
  if (fsrGet().agg.channelsInContact > 0) { say(c, false, "Capture background refused: an FSR shows contact - empty the gripper"); return; }
  const TofSnapshot t = tofGet();
  if (t.online != 3) { say(c, false, "Capture background refused: all three SEN0628 must be online"); return; }
  static uint16_t bg[3][64];
  static int16_t zero[3][64], d[3][64];
  memset(zero, 0, sizeof zero);
  backgroundAt(sweepModel(), sweepSamples(), servoGet().fb.position, zero, bg);
  int changed = 0;
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) {
    d[s][z] = 0;
    if ((sweepModel().fingerZones[s] >> z) & 1) continue;
    const uint16_t v = t.s[s].mm[z];
    if (v == 0 || v >= 4000 || bg[s][z] == 0) continue;
    d[s][z] = (int16_t)((int)v - (int)bg[s][z]);
    if (d[s][z] > 15 || d[s][z] < -15) changed++;
  }
  if (!bgDeltaSave(d)) { say(c, false, "Capture background: could not save /bgdelta.bin"); return; }
  smootherReset(c.sm);
  say(c, true, "background captured: %d zones differ from the sweep by more than 15 mm", changed);
}

// ---- Phase 4a: one log row per FSR sample (100 Hz) ----
int16_t toMilli(float v) {
  if (!(v == v)) return INT16_MIN;
  const float x = v * 1000.0f;
  return (int16_t)(x > 32767 ? 32767 : x < -32767 ? -32767 : x);
}
int16_t toTenth(float v, bool ok) {
  if (!ok || !(v == v)) return INT16_MIN;
  const float x = v * 10.0f;
  return (int16_t)(x > 32767 ? 32767 : x < -32767 ? -32767 : x);
}

void logStep(Ctl& c, uint32_t now) {
  if (!gLog.rows) return;
  const State st = c.fsm.state();
  LogRow r{};
  r.tMs = now; r.state = (uint8_t)st; r.armed = c.fsm.armed();
  r.step = st == State::PRE_TOUCH ? (uint8_t)((uint8_t)c.fsm.preTouchStep() + 1) : 0;
  const ServoSnapshot sv = servoGet();
  r.pos = sv.fb.position; r.speed = sv.fb.speed;
  r.currentMa = (int16_t)(sv.fb.current * 6.5f);
  r.loadPermille = (int16_t)(sv.fb.load * 1000.0f);
  const FsrSnapshot fs = fsrGet();
  for (int i = 0; i < 6; i++) {
    const int col = FSR_CHANNEL_NUMBER[i] - 1;           // printed order FSR1..FSR6
    r.fmN[col] = toMilli(fs.force[i]);
    r.rmN[col] = toMilli(fs.forceRaw[i]);
  }
  for (int f = 0; f < 3; f++) if (fs.contacts.stable[f]) r.contacts |= (uint8_t)(1u << f);
  r.tofSeq = (uint16_t)(c.objOk[0] + c.objOk[1] + c.objOk[2]);
  const ObjectMeasure m = smoothed(c.sm, now);
  r.objValid = m.valid;
  r.width10 = toTenth(m.widthMm, m.valid); r.range10 = toTenth(m.rangeMm, m.valid); r.height10 = toTenth(m.heightMm, m.valid);
  for (int s = 0; s < 3; s++) r.rad10[s] = toTenth(m.sensorRadiusMm[s], m.valid);
  r.cls = c.pr.plan.valid ? (uint8_t)c.pr.plan.cls : 255;
  r.shape = (uint8_t)m.shape;
  // Events: one per row; STATE first, then grasp reached, then the first contact per finger.
  if (st != c.logState) {
    if (c.logState == State::GRASP && st == State::MONITOR) c.pendGrasp = true;
    if (st == State::START) for (auto& f : c.logFirst) f = FirstContact::NONE;
    c.logState = st;
    r.ev = LogEvent::STATE; r.evArg = (uint8_t)st;
  }
  for (int f = 0; f < 3; f++)
    if (c.logFirst[f] == FirstContact::NONE && c.first[f] != FirstContact::NONE) { c.logFirst[f] = c.first[f]; c.pendContact |= (uint8_t)(1u << f); }
  if (r.ev == LogEvent::NONE && c.pendGrasp) { r.ev = LogEvent::GRASP_REACHED; c.pendGrasp = false; }
  if (r.ev == LogEvent::NONE && c.pendContact)
    for (int f = 0; f < 3; f++)
      if (c.pendContact & (1u << f)) {
        c.pendContact &= (uint8_t)~(1u << f);
        r.ev = LogEvent::CONTACT; r.evArg = (uint8_t)(f + 1); r.evArg2 = (uint8_t)c.logFirst[f];
        break;
      }
  logPush(gLog, r);
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
  s.loopMaxUs = c.loopMaxUs; s.posJumps = c.posJumps; s.fsrSampleUsMax = c.fsrUsMax;
  s.controlStackFree = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);   // this task (bytes on ESP32)
  s.homed = c.homed; s.homing = c.homing; s.homeResidual = c.homeResidual;
  s.sweeping = c.sweeping; s.sweepN = c.swN; s.sweepLoaded = sweepLoaded();
  s.estValid = c.est.valid && !c.homed; s.estPos = c.est.pos;
  s.estTurn = c.cfg.touchValid ? turnOf(c.est.pos, c.cfg.touchRef) : 0;
  snprintf(s.homeMsg, sizeof(s.homeMsg), "%s", c.homeMsg);
  s.obj = smoothed(c.sm, millis());
  s.objDetectedValid = c.pr.detectedValid; s.objDetected = (uint8_t)c.pr.detected;
  s.planValid = c.pr.plan.valid; s.planCls = (uint8_t)c.pr.plan.cls;
  s.planApproach = c.pr.plan.approach; s.planGoal = c.pr.plan.approachGoal;
  snprintf(s.planWhy, sizeof(s.planWhy), "%s", c.pr.why ? c.pr.why : "");
  s.override_ = (uint8_t)c.ov; s.delicate = c.delicate; s.approachOnly = c.approachOnly;
  s.preTouchStep = (uint8_t)c.fsm.preTouchStep();
  s.latchedCls = (uint8_t)c.fsm.plan().cls;
  for (int f = 0; f < 3; f++) s.firstContact[f] = (uint8_t)c.first[f];
  const Pattern pat = patternOf(c.first);
  s.pattern = (uint8_t)pat;
  s.patternOk = patternConsistent(pat, c.fsm.plan().cls);
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
  bootStop(*c);                        // stop a still-running servo before anything slow
  tofCalibLoad(c->cal);                // SEN0628 calibration saved by the 3.0.3 3D page
  sweepStoreBegin();                   // LittleFS (may format on first use) + /sweep.bin
  {   // phase 4a data log: ~5 min at 100 Hz in PSRAM (small fallback in internal RAM)
    LogRow* st = (LogRow*)ps_malloc(sizeof(LogRow) * LOG_CAP);
    uint32_t cap = LOG_CAP;
    if (!st) { cap = 3000; st = (LogRow*)malloc(sizeof(LogRow) * cap); }
    if (st) logInit(gLog, st, cap);
  }
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
      if (c->homing) homeStop(*c, false, "aborted by E-stop");
      if (c->sweeping) sweepStop(*c, false, "aborted by E-stop");
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
    if ((uint32_t)(now - lastFsr) >= 10) {
      lastFsr = now;
      const uint32_t t0 = micros();
      fsrSample(c->cfg.contactForceN, c->cfg.contactSamples, c->cfg.fsrEnabled, filterParamsOf(c->cfg));
      const uint32_t dt = micros() - t0;
      if (dt > c->fsrUsMaxWin) c->fsrUsMaxWin = dt;
      logStep(*c, now);
    }
    if (!modeRead || (uint32_t)(now - lastMode) >= 1000) { modeRead = true; lastMode = now; servoReadModeAndPhase(); }

    homeUpdate(*c, now);
    sweepUpdate(*c, now);
    estimateUpdate(*c, now);
    checkUpdate(*c, now);
    objectUpdate(*c, now);
    planUpdate(*c, now);
    if (c->pendingOpen && cmd == Cmd::NONE) { c->pendingOpen = false; cmd = Cmd::OPEN; }
    FsmInputs fin = buildInputs(now, cmd);
    fin.homed = c->homed;
    fin.plan = c->pr.plan; fin.planWhy = c->pr.why; fin.approachOnly = c->approachOnly;
    MotorAction a = c->fsm.step(fin);
    contactUpdate(*c);
    if (a.kind != MotorAction::NONE && !applyAction(a)) {
      c->fsm.motorFailed(servoGet().lastError);
      servoStop();
    }

    {   // loop timing diagnostics: max iteration time over 5 s windows
      const uint32_t dt = micros() - loopStartUs;
      if (dt > c->loopMaxWin) c->loopMaxWin = dt;
      if ((uint32_t)(now - c->loopWinStart) >= 5000) {
        c->loopMaxUs = c->loopMaxWin; c->loopMaxWin = 0;
        c->fsrUsMax = c->fsrUsMaxWin; c->fsrUsMaxWin = 0;
        c->loopWinStart = now;
      }
    }
    publish(*c);
    vTaskDelay(1);
  }
}
