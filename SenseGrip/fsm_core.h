// Gripper state machine and "why is it blocked" list. Pure C++.
//
// The glue (fsm.cpp) builds FsmInputs from the sensor snapshots, calls step()
// every ~1 ms and applies the returned MotorAction:
//   RUN  -> servoRunAt(speed, torque)        speed > 0 = closing
//   HOLD -> servoHoldHere(torque, speed)
//   MOVE -> servoMoveTo(position, speed, torque)
//   STOP -> servoStop()
// If applying an action fails it calls motorFailed(err).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "config_core.h"

enum class State : uint8_t { IDLE, START, PRE_TOUCH, GRASP, MONITOR, OPEN, FAULT };
const char* stateName(State s);

enum class Cmd : uint8_t { NONE, ARM, DISARM, OPEN, ESTOP, RESET, TIGHTEN };
// Phase 4b tighten: MONITOR -> GRASP at a higher target, back to MONITOR when reached or failed.
enum class TightenEvent : uint8_t { NONE, STARTED, DONE, FAILED };
constexpr uint8_t TIGHTEN_MAX = 2;            // per grasp
constexpr float TIGHTEN_GAIN = 1.25f;         // new target = previous x gain ...
constexpr float TIGHTEN_CAP = 0.9f;           // ... but at most this fraction of Max force

// What to grasp and how (built by plan_core from the SEN0628 measurement, the class profile and
// the operator's override / delicate switches). Latched when START hands over to PRE_TOUCH.
struct GraspPlan {
  bool valid;            // usable for a grasp (measured, or override / delicate set)
  ObjClass cls;          // effective class
  bool approach;         // a fast-approach goal exists
  int32_t approachGoal;  // logical position of the pre-touch point
  ClassProfile prof;
};
enum class PreTouchStep : uint8_t { APPROACH, SLOW };
constexpr uint32_t START_STABLE_MS = 300;    // same class this long before moving (spec §7.3)
constexpr uint32_t START_TIMEOUT_MS = 3000;  // ... or START gives up
constexpr uint32_t APPROACH_STALL_MS = 400;  // no progress this long ends the fast approach
// OPEN: the servo can stop a little short of its goal (seen 11-20 counts). Stopped (< 5 counts of
// progress) for OPEN_SETTLE_MS within OPEN_SETTLE_COUNTS of OPEN zero counts as arrived.
constexpr uint32_t OPEN_SETTLE_MS = 500;
constexpr int32_t OPEN_SETTLE_COUNTS = 100;
// Slow close and GRASP move at 15 / 3-10 speed units (~50 counts/s each): their timeout is the
// configured one, or 1.5 x the time to cover the distance left to the close limit + 2 s if longer.
constexpr uint32_t COUNTS_PER_S_PER_SPEED = 50;

struct FsmInputs {
  uint32_t nowMs; Cmd cmd;
  bool servoOnline; uint32_t servoAgeMs; uint8_t servoStatus; bool servoFullAngle; bool servoModeKnown; uint8_t servoMode;
  int32_t position;
  bool homed;             // position frame valid since this boot (Home or sweep done)
  bool fsrCalibrated[6]; float fsrForce[6]; bool anyFingerContact; float avgContactForce;
  uint8_t channelsInContact; float maxForce; uint32_t fsrSeq;
  uint8_t tofOnline;      // number (or mask) of online ToF sensors; 0 = none
  bool objectPresent;     // already debounced upstream (presence hold)
  GraspPlan plan; const char* planWhy;   // planWhy: reason when !plan.valid (may be null)
  bool approachOnly;      // test switch: stop in MONITOR at the pre-touch position
  bool slipEvent;         // slip detector fired on this step (phase 4b)
  bool tightenEnabled;    // config: tighten on slip
};

struct MotorAction {
  enum Kind : uint8_t { NONE, RUN, HOLD, MOVE, STOP } kind;
  int16_t speed; int32_t position; uint16_t torque;
};

// Blocker ids (stable, for the dashboard).
enum BlockerId : uint8_t {
  BLK_SERVO_OFFLINE = 1, BLK_SERVO_STATUS, BLK_SERVO_MODE, BLK_FULL_ANGLE, BLK_NOT_HOMED, BLK_LIMITS, BLK_NOT_OPENED,
  BLK_FSR_UNCALIBRATED, BLK_FSR_LOADED, BLK_NO_TOF, BLK_NOT_ARMED,
};
struct Blocker { uint8_t id; char text[64]; };

class Fsm {
public:
  explicit Fsm(const GripConfig& c);
  void setConfig(const GripConfig& c);
  MotorAction step(const FsmInputs& in);   // at most one action per call
  // -> FAULT "servo command failed: <err>"; STOP on next step(). If it fails again
  // while in FAULT (e.g. the STOP itself), STOP is retried every 100 ms until one succeeds.
  void motorFailed(const char* err);
  State state() const { return state_; }
  bool armed() const { return armed_; }
  bool openedSinceBoot() const { return openedSinceBoot_; }
  const char* reason() const { return reason_; }   // last transition / refusal text
  State faultFrom() const { return faultFrom_; }
  const char* faultReason() const { return faultReason_; } // what caused the last FAULT entry
  uint8_t blockers(Blocker* out, uint8_t cap) const;  // from last step; "not armed" last
  // Forget that the jaw reached OPEN (after a jog, new limits or a direction change):
  // the "not at OPEN" blocker returns until Open completes again.
  void invalidateOpened() { openedSinceBoot_ = false; }
  PreTouchStep preTouchStep() const { return preStep_; }
  const GraspPlan& plan() const { return plan_; }      // latched at START -> PRE_TOUCH
  uint8_t tightenCount() const { return tightenCount_; }
  bool tightening() const { return tightening_; }
  TightenEvent takeTightenEvent() { const TightenEvent e = tevent_; tevent_ = TightenEvent::NONE; return e; }

  static constexpr uint8_t MAX_BLOCKERS = 20;
  static constexpr uint8_t MAX_REPORTED = MAX_BLOCKERS + 1;   // + "not armed"

private:

  MotorAction run(const FsmInputs& in);        // the per-state switch
  MotorAction enter(State s, const char* why);  // entry actions
  MotorAction handleCommand(const FsmInputs& in);
  MotorAction onOpen(const FsmInputs& in);
  MotorAction onReset(const FsmInputs& in);
  MotorAction onDisarm();
  MotorAction startUpdate(const FsmInputs& in);
  MotorAction tryTighten(const char* why);
  MotorAction tightenEnd(bool ok, const char* why);
  void onArm();
  void computeBlockers(const FsmInputs& in);
  void addBlocker(uint8_t id, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
  const char* safetyTrip(const FsmInputs& in);
  bool graspReached(const FsmInputs& in);
  bool pastCloseLimit(int32_t pos) const;
  bool outsideTravel(int32_t pos) const;
  bool timedOut(const FsmInputs& in, uint32_t limitMs) const;
  uint32_t travelTimeoutMs(uint32_t baseMs, uint16_t speed) const;
  void setReason(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

  GripConfig cfg_;
  State state_ = State::IDLE;
  State faultFrom_ = State::IDLE;
  bool armed_ = false;
  bool openedSinceBoot_ = false;
  uint32_t nowMs_ = 0;
  uint32_t entryMs_ = 0;
  uint32_t fsrSeq_ = 0;
  uint32_t graspSeq_ = 0;
  uint8_t graspCount_ = 0;
  GraspPlan plan_ = {};
  PreTouchStep preStep_ = PreTouchStep::SLOW;
  bool planEverValid_ = false; bool haveCls_ = false; bool startFirst_ = false; ObjClass stableCls_ = ObjClass::SMALL; uint32_t clsSinceMs_ = 0;
  int32_t lastPos_ = 0; int32_t progPos_ = 0; uint32_t progMs_ = 0;
  bool approachOnly_ = false;
  uint32_t stepTimeoutMs_ = 0;            // PRE_TOUCH slow step / GRASP: distance-scaled timeout
  bool tightening_ = false; uint8_t tightenCount_ = 0; TightenEvent tevent_ = TightenEvent::NONE;
  bool heldByGrasp_ = false;              // MONITOR reached by force (not an approach-only stop)
  bool stopPending_ = false;               // STOP queued by motorFailed()
  uint32_t stopDueMs_ = 0;                 // ... not before this time
  char reason_[96];
  char faultReason_[96] = "";
  char scratch_[96];                       // formatted safety-trip / abort text
  Blocker blockers_[MAX_BLOCKERS];
  uint8_t nBlockers_ = 0;
};
