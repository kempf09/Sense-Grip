#include "check.h"
#include "fsm_core.h"
#include <cstring>

static GripConfig cfg() {
  GripConfig c = defaultConfig();
  c.openZero = 1968; c.closeLimit = 19102; c.closePositive = true; c.limitsValid = true;
  c.maxForceN = 3.0f;   // the force-limit tests were written against 3 N
  return c;
}

// Healthy inputs: servo online at openZero, all FSRs calibrated and unloaded, ToF online.
static FsmInputs ready(uint32_t now = 0) {
  FsmInputs in{};
  in.nowMs = now; in.cmd = Cmd::NONE;
  in.servoOnline = true; in.servoAgeMs = 5; in.servoStatus = 0;
  in.servoFullAngle = true; in.servoModeKnown = true; in.servoMode = 0; in.homed = true; in.position = 1968;
  for (int i = 0; i < 6; i++) { in.fsrCalibrated[i] = true; in.fsrForce[i] = 0; }
  in.tofOnline = 3; in.objectPresent = false;
  // Measured medium object, no fast approach: slow to contact at 15, grasp 1.0 N at speed 10 / 200.
  in.plan = GraspPlan{true, ObjClass::MEDIUM, false, 0, ClassProfile{10, 1.0f, 200, 10}};
  return in;
}

static bool hasBlocker(const Fsm& f, const char* needle) {
  Blocker b[24];
  uint8_t n = f.blockers(b, 24);
  for (uint8_t i = 0; i < n; i++) if (std::strstr(b[i].text, needle)) return true;
  return false;
}

static MotorAction cmd(Fsm& f, FsmInputs in, Cmd c) { in.cmd = c; return f.step(in); }

// Fresh FSM that has been sent OPEN once and reached openZero (IDLE, disarmed).
static Fsm openedFsm() {
  Fsm f(cfg());
  FsmInputs in = ready(0);
  in.position = 5000;
  f.step(in);
  MotorAction a = cmd(f, in, Cmd::OPEN);
  CHECK(f.state() == State::OPEN);
  CHECK(a.kind == MotorAction::MOVE);
  CHECK_EQ(a.position, 1968);
  in = ready(10);
  a = f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::HOLD);
  CHECK(f.openedSinceBoot());
  return f;
}

// openedFsm, armed, triggered at t0 - 300 (START), class stable: PRE_TOUCH entered at t0.
static Fsm preTouchFsm(uint32_t t0) {
  Fsm f = openedFsm();
  cmd(f, ready(t0 - START_STABLE_MS), Cmd::ARM);
  FsmInputs in = ready(t0 - START_STABLE_MS);
  in.objectPresent = true;
  f.step(in);
  CHECK(f.state() == State::START);
  in.nowMs = t0;
  f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  return f;
}

static Fsm graspFsm(uint32_t t0) {
  Fsm f = preTouchFsm(t0);
  FsmInputs in = ready(t0);
  in.anyFingerContact = true; in.fsrSeq = 100; in.position = 8000;
  f.step(in);
  CHECK(f.state() == State::GRASP);
  return f;
}

TEST(fsm_boot_blocks_until_opened) {
  Fsm f(cfg());
  f.step(ready());
  CHECK(f.state() == State::IDLE);
  CHECK(hasBlocker(f, "press Open"));
  CHECK(hasBlocker(f, "not armed"));
  cmd(f, ready(1), Cmd::ARM);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "press Open") != nullptr);
}

TEST(fsm_not_armed_listed_last) {
  Fsm f = openedFsm();
  FsmInputs in = ready(20);
  in.tofOnline = 0;
  f.step(in);
  Blocker b[24];
  uint8_t n = f.blockers(b, 24);
  CHECK_EQ(n, (uint8_t)2);
  CHECK(std::strcmp(b[0].text, "no ToF sensor online") == 0);
  CHECK(std::strcmp(b[1].text, "not armed") == 0);
  cmd(f, ready(21), Cmd::ARM);
  CHECK(f.armed());
  CHECK(!hasBlocker(f, "not armed"));
}

TEST(fsm_arm_then_trigger_runs_approach) {
  Fsm f = openedFsm();
  cmd(f, ready(20), Cmd::ARM);
  CHECK(f.armed());
  FsmInputs in = ready(21);
  in.objectPresent = true;
  in.plan.approach = true; in.plan.approachGoal = 9000;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::START);
  CHECK(a.kind == MotorAction::NONE);
  in.nowMs = 21 + START_STABLE_MS - 1;
  CHECK(f.step(in).kind == MotorAction::NONE);
  CHECK(f.state() == State::START);
  in.nowMs = 21 + START_STABLE_MS;
  a = f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  CHECK(a.kind == MotorAction::MOVE);
  CHECK_EQ(a.position, 9000);
  CHECK_EQ(a.speed, 61);
  CHECK_EQ(a.torque, 200);
  CHECK(f.preTouchStep() == PreTouchStep::APPROACH);
}

TEST(fsm_arm_with_object_already_present_waits_hold) {
  Fsm f = openedFsm();
  cmd(f, ready(20), Cmd::ARM);
  for (uint32_t t = 21; t < 500; t++) f.step(ready(t));
  CHECK(f.state() == State::IDLE);
  CHECK(f.armed());
  FsmInputs in = ready(500);
  in.objectPresent = true;
  f.step(in);
  CHECK(f.state() == State::START);
}

TEST(fsm_start_bounces_on_blocker) {
  Fsm f = openedFsm();
  cmd(f, ready(20), Cmd::ARM);
  FsmInputs in = ready(21);
  in.objectPresent = true;
  f.step(in);
  CHECK(f.state() == State::START);
  in.fsrCalibrated[0] = false;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "FSR 2 not calibrated") != nullptr);
}

TEST(fsm_contact_switches_to_grasp_speed) {
  Fsm f = preTouchFsm(100);
  FsmInputs in = ready(150);
  in.position = 6000;
  CHECK(f.step(in).kind == MotorAction::NONE);
  in.anyFingerContact = true;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::GRASP);
  CHECK(a.kind == MotorAction::RUN);
  CHECK_EQ(a.speed, 10);
  CHECK_EQ(a.torque, 200);
}

TEST(fsm_grasp_needs_samples_and_channels) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(110);
  in.anyFingerContact = true; in.position = 8000;
  in.avgContactForce = 1.2f; in.channelsInContact = 1;
  for (uint32_t s = 101; s < 120; s++) { in.fsrSeq = s; in.nowMs++; f.step(in); }
  CHECK(f.state() == State::GRASP);

  in.channelsInContact = 2;
  in.fsrSeq = 200; CHECK(f.step(in).kind == MotorAction::NONE);
  f.step(in); f.step(in);                     // same seq: not counted again
  in.fsrSeq = 201; CHECK(f.step(in).kind == MotorAction::NONE);
  CHECK(f.state() == State::GRASP);
  in.fsrSeq = 202;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::MONITOR);
  CHECK(a.kind == MotorAction::HOLD);
  CHECK_EQ(a.torque, 200);
  CHECK_EQ(a.speed, 10);
  in.fsrSeq = 203;
  CHECK(f.step(in).kind == MotorAction::NONE);  // MONITOR: no automatic action
  CHECK(f.state() == State::MONITOR);
}

TEST(fsm_grasp_failed_sample_resets_count) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(110);
  in.anyFingerContact = true; in.position = 8000;
  in.avgContactForce = 1.2f; in.channelsInContact = 2;
  in.fsrSeq = 101; f.step(in);
  in.fsrSeq = 102; f.step(in);
  in.avgContactForce = 0.5f; in.fsrSeq = 103; f.step(in);
  in.avgContactForce = 1.2f;
  in.fsrSeq = 104; f.step(in);
  in.fsrSeq = 105; f.step(in);
  CHECK(f.state() == State::GRASP);
  in.fsrSeq = 106; f.step(in);
  CHECK(f.state() == State::MONITOR);
}

TEST(fsm_close_limit_without_contact_faults) {
  Fsm f = preTouchFsm(100);
  FsmInputs in = ready(200);
  in.position = 19102 - 9;                       // just outside the 8-count tolerance
  f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  in.position = 19102 - 8;                       // within tolerance of the close limit
  MotorAction a = f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(a.kind == MotorAction::STOP);
  CHECK(std::strcmp(f.reason(), "no contact before close limit") == 0);
  CHECK(f.faultFrom() == State::PRE_TOUCH);
}

TEST(fsm_close_limit_mirrored_when_closing_negative) {
  GripConfig c = cfg();
  c.openZero = 19102; c.closeLimit = 1968; c.closePositive = false;
  Fsm f(c);
  FsmInputs in = ready(0);
  in.position = 19102;
  cmd(f, in, Cmd::OPEN);
  f.step(in);
  CHECK(f.state() == State::IDLE);
  cmd(f, in, Cmd::ARM);
  in.objectPresent = true;
  f.step(in); in.nowMs += START_STABLE_MS; f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  in.position = 1968 + 9; f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  in.position = 1968 + 8; f.step(in);
  CHECK(f.state() == State::FAULT);
}

TEST(fsm_grasp_close_limit_faults) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(120);
  in.anyFingerContact = true; in.position = 19200;
  f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(std::strcmp(f.reason(), "target force not reached before close limit") == 0);
}

TEST(fsm_pretouch_timeout_faults) {
  Fsm f = preTouchFsm(0xFFFFFF00u);           // entry near uint32 wrap
  FsmInputs in = ready(0xFFFFFF00u + 7999);
  in.position = 6000;
  f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  in.nowMs = 0xFFFFFF00u + 8000;              // wraps past zero
  MotorAction a = f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(a.kind == MotorAction::STOP);
  CHECK(std::strstr(f.reason(), "PRE_TOUCH") != nullptr);
}

TEST(fsm_force_limit_faults_from_grasp) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(120);
  in.anyFingerContact = true; in.position = 8000; in.maxForce = 3.1f;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(a.kind == MotorAction::STOP);
  CHECK(f.faultFrom() == State::GRASP);
  CHECK(std::strcmp(f.reason(), "FSR force limit") == 0);
  CHECK(!f.armed());
  CHECK(!f.openedSinceBoot());
}

TEST(fsm_global_checks_while_moving) {
  {
    Fsm f = preTouchFsm(100);
    FsmInputs in = ready(120); in.position = 6000; in.servoAgeMs = 101;
    f.step(in);
    CHECK(f.state() == State::FAULT);
  }
  {
    Fsm f = preTouchFsm(100);
    FsmInputs in = ready(120); in.position = 6000; in.servoStatus = 0x20;
    f.step(in);
    CHECK(f.state() == State::FAULT);
    CHECK(std::strstr(f.reason(), "servo status") != nullptr);
  }
  {
    Fsm f = preTouchFsm(100);
    FsmInputs in = ready(120); in.position = 1968 - 201;
    f.step(in);
    CHECK(f.state() == State::FAULT);
    CHECK(std::strcmp(f.reason(), "position outside travel limits") == 0);
  }
}

TEST(fsm_open_from_monitor_returns_idle_disarmed) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(120);
  in.anyFingerContact = true; in.position = 8000;
  in.avgContactForce = 1.5f; in.channelsInContact = 3;
  for (uint32_t s = 101; s <= 103; s++) { in.fsrSeq = s; f.step(in); }
  CHECK(f.state() == State::MONITOR);
  CHECK(f.armed());
  MotorAction a = cmd(f, in, Cmd::OPEN);
  CHECK(f.state() == State::OPEN);
  CHECK(a.kind == MotorAction::MOVE);
  CHECK_EQ(a.position, 1968);
  CHECK_EQ(a.speed, 61);
  CHECK_EQ(a.torque, 260);
  CHECK(cmd(f, in, Cmd::OPEN).kind == MotorAction::NONE);  // OPEN in OPEN: no-op
  in.position = 1980; in.avgContactForce = 0; in.anyFingerContact = false;
  CHECK(f.step(in).kind == MotorAction::NONE);
  in.position = 1976;
  a = f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::HOLD);
  CHECK_EQ(a.torque, 260);
  CHECK_EQ(a.speed, 61);
  CHECK(!f.armed());
}

TEST(fsm_open_timeout_faults) {
  Fsm f(cfg());
  FsmInputs in = ready(1000);
  in.position = 5000;
  cmd(f, in, Cmd::OPEN);
  in.nowMs = 1000 + 15000;
  f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(f.faultFrom() == State::OPEN);
  CHECK(std::strstr(f.reason(), "OPEN") != nullptr);
}

TEST(fsm_open_refused_without_servo_or_limits) {
  Fsm f(defaultConfig());
  cmd(f, ready(), Cmd::OPEN);
  CHECK(f.state() == State::IDLE);
  CHECK(std::strstr(f.reason(), "not set") != nullptr);
  Fsm g(cfg());
  FsmInputs in = ready(); in.servoOnline = false;
  cmd(g, in, Cmd::OPEN);
  CHECK(g.state() == State::IDLE);
  CHECK(std::strstr(g.reason(), "servo offline") != nullptr);
}

TEST(fsm_open_refused_in_fault) {
  Fsm f = openedFsm();
  cmd(f, ready(20), Cmd::ESTOP);
  MotorAction a = cmd(f, ready(21), Cmd::OPEN);
  CHECK(f.state() == State::FAULT);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(std::strstr(f.reason(), "Reset first") != nullptr);
}

TEST(fsm_reset_blocked_while_cause_active) {
  Fsm f = preTouchFsm(100);
  FsmInputs in = ready(120); in.position = 6000; in.servoStatus = 0x04;
  f.step(in);
  CHECK(f.state() == State::FAULT);
  cmd(f, in, Cmd::RESET);
  CHECK(f.state() == State::FAULT);
  CHECK(std::strstr(f.reason(), "Reset refused") != nullptr);
  in.servoStatus = 0; in.servoOnline = false;
  cmd(f, in, Cmd::RESET);
  CHECK(f.state() == State::FAULT);
  CHECK(std::strstr(f.reason(), "servo offline") != nullptr);
  in.servoOnline = true;
  MotorAction a = cmd(f, in, Cmd::RESET);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(!f.armed());
  CHECK(hasBlocker(f, "press Open"));       // torque was off: Open needed again
}

TEST(fsm_reset_ignored_outside_fault) {
  Fsm f = openedFsm();
  cmd(f, ready(20), Cmd::RESET);
  CHECK(f.state() == State::IDLE);
  CHECK(std::strstr(f.reason(), "Reset ignored") != nullptr);
}

TEST(fsm_motor_failure_faults) {
  Fsm f = preTouchFsm(100);
  f.motorFailed("timeout");
  CHECK(f.state() == State::FAULT);
  CHECK(f.faultFrom() == State::PRE_TOUCH);
  CHECK(std::strcmp(f.reason(), "servo command failed: timeout") == 0);
  FsmInputs in = ready(120); in.position = 6000;
  CHECK(f.step(in).kind == MotorAction::STOP);
  CHECK(f.step(in).kind == MotorAction::NONE);
}

TEST(fsm_estop_from_any_state) {
  {
    Fsm f(cfg());
    MotorAction a = cmd(f, ready(), Cmd::ESTOP);
    CHECK(f.state() == State::FAULT && a.kind == MotorAction::STOP);
    CHECK(f.faultFrom() == State::IDLE);
    CHECK(std::strcmp(f.reason(), "E-stop") == 0);
  }
  {
    Fsm f = preTouchFsm(100);
    FsmInputs in = ready(120); in.position = 6000;
    MotorAction a = cmd(f, in, Cmd::ESTOP);
    CHECK(f.state() == State::FAULT && a.kind == MotorAction::STOP);
    CHECK(f.faultFrom() == State::PRE_TOUCH);
    a = cmd(f, in, Cmd::ESTOP);               // again while in FAULT: still STOP
    CHECK(a.kind == MotorAction::STOP);
    CHECK(f.faultFrom() == State::PRE_TOUCH);
  }
  {
    Fsm f(cfg());
    FsmInputs in = ready(0); in.position = 5000;
    cmd(f, in, Cmd::OPEN);
    MotorAction a = cmd(f, in, Cmd::ESTOP);
    CHECK(f.state() == State::FAULT && a.kind == MotorAction::STOP);
    CHECK(f.faultFrom() == State::OPEN);
  }
}

TEST(fsm_disarm_any_state) {
  {  // IDLE: just clears the flag
    Fsm f = openedFsm();
    cmd(f, ready(20), Cmd::ARM);
    CHECK(f.armed());
    MotorAction a = cmd(f, ready(21), Cmd::DISARM);
    CHECK(!f.armed());
    CHECK(f.state() == State::IDLE);
    CHECK(a.kind == MotorAction::NONE);
  }
  {  // MONITOR: keeps holding, flag cleared
    Fsm f = graspFsm(100);
    FsmInputs in = ready(120);
    in.anyFingerContact = true; in.position = 8000;
    in.avgContactForce = 1.5f; in.channelsInContact = 3;
    for (uint32_t s = 101; s <= 103; s++) { in.fsrSeq = s; f.step(in); }
    CHECK(f.state() == State::MONITOR);
    MotorAction a = cmd(f, in, Cmd::DISARM);
    CHECK(!f.armed());
    CHECK(f.state() == State::MONITOR);
    CHECK(a.kind == MotorAction::NONE);
  }
  {  // FAULT: stays FAULT
    Fsm f = preTouchFsm(100);
    FsmInputs in = ready(120); in.position = 6000;
    cmd(f, in, Cmd::ESTOP);
    MotorAction a = cmd(f, in, Cmd::DISARM);
    CHECK(f.state() == State::FAULT);
    CHECK(a.kind == MotorAction::NONE);
  }
  {  // GRASP: opens
    Fsm f = graspFsm(100);
    FsmInputs in = ready(120); in.anyFingerContact = true; in.position = 8000;
    MotorAction a = cmd(f, in, Cmd::DISARM);
    CHECK(!f.armed());
    CHECK(f.state() == State::OPEN);
    CHECK(a.kind == MotorAction::MOVE);
    CHECK_EQ(a.position, 1968);
    CHECK(std::strcmp(f.reason(), "Disarmed during GRASP — opening") == 0);
  }
  {  // START: opens too (never moved, but OPEN is harmless and consistent)
    Fsm f = openedFsm();
    cmd(f, ready(20), Cmd::ARM);
    FsmInputs in = ready(21); in.objectPresent = true;
    f.step(in);
    CHECK(f.state() == State::START);
    MotorAction a = cmd(f, in, Cmd::DISARM);
    CHECK(f.state() == State::OPEN);
    CHECK(a.kind == MotorAction::MOVE);
    CHECK(std::strstr(f.reason(), "during START") != nullptr);
  }
}

TEST(fsm_disarm_in_pretouch_opens) {
  Fsm f = preTouchFsm(100);
  FsmInputs in = ready(120); in.position = 6000;
  MotorAction a = cmd(f, in, Cmd::DISARM);
  CHECK(!f.armed());
  CHECK(f.state() == State::OPEN);
  CHECK(a.kind == MotorAction::MOVE);
  CHECK_EQ(a.position, 1968);
  CHECK_EQ(a.speed, 61);
  CHECK_EQ(a.torque, 260);
  CHECK(std::strcmp(f.reason(), "Disarmed during PRE_TOUCH — opening") == 0);
  in.position = 1970;                          // arrives at OPEN: IDLE, disarmed
  CHECK(f.step(in).kind == MotorAction::HOLD);
  CHECK(f.state() == State::IDLE);
  CHECK(!f.armed());
}

TEST(fsm_invalidate_opened_brings_blocker_back) {
  Fsm f = openedFsm();
  f.step(ready(20));
  CHECK(!hasBlocker(f, "press Open"));
  f.invalidateOpened();
  CHECK(!f.openedSinceBoot());
  f.step(ready(21));
  CHECK(hasBlocker(f, "press Open"));
  cmd(f, ready(22), Cmd::ARM);
  CHECK(!f.armed());
  cmd(f, ready(23), Cmd::OPEN);                // Open again clears it
  f.step(ready(24));                           // reaches OPEN -> IDLE
  CHECK(f.openedSinceBoot());
  f.step(ready(25));                           // blockers are computed at the start of step()
  CHECK(!hasBlocker(f, "press Open"));
}

TEST(fsm_blocker_text_not_truncated) {
  GripConfig c = cfg();
  c.closeLimit = 1000;                         // wrong side of openZero
  Fsm f(c);
  f.step(ready());
  CHECK(hasBlocker(f, "close limit is not on the closing side of OPEN zero"));
  Blocker b[Fsm::MAX_REPORTED];
  uint8_t n = f.blockers(b, Fsm::MAX_REPORTED);
  CHECK(n <= Fsm::MAX_REPORTED);
  for (uint8_t i = 0; i < n; i++) CHECK(std::strlen(b[i].text) < sizeof b[i].text - 1);
}

TEST(fsm_all_blockers_fit_reported_capacity) {
  Fsm f(defaultConfig());
  FsmInputs in = ready();
  in.servoOnline = false; in.servoStatus = 0x2F; in.servoFullAngle = false; in.tofOnline = 0;
  for (int i = 0; i < 6; i++) in.fsrCalibrated[i] = false;
  f.step(in);
  Blocker b[Fsm::MAX_REPORTED];
  uint8_t n = f.blockers(b, Fsm::MAX_REPORTED);
  CHECK_EQ(n, 13);                             // 12 blockers + "not armed"
  CHECK(b[n - 1].id == BLK_NOT_ARMED);
}

TEST(fsm_uncalibrated_fsr_named) {
  Fsm f = openedFsm();
  FsmInputs in = ready(20);
  in.fsrCalibrated[2] = false;
  f.step(in);
  CHECK(hasBlocker(f, "FSR 3 not calibrated"));
  cmd(f, in, Cmd::ARM);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "FSR 3 not calibrated") != nullptr);
}

TEST(fsm_loaded_fsr_only_blocks_in_idle) {
  Fsm f = openedFsm();
  FsmInputs in = ready(20);
  in.fsrForce[0] = 0.25f;
  f.step(in);
  CHECK(hasBlocker(f, "FSR 2 loaded at rest"));
  Fsm g = graspFsm(100);
  in = ready(120); in.anyFingerContact = true; in.position = 8000; in.fsrForce[0] = 1.0f;
  g.step(in);
  CHECK(!hasBlocker(g, "loaded at rest"));
}

TEST(fsm_servo_blockers) {
  Fsm f(cfg());
  FsmInputs in = ready();
  in.servoAgeMs = 101; in.servoStatus = 0x01; in.servoFullAngle = false;
  f.step(in);
  CHECK(hasBlocker(f, "servo offline"));
  CHECK(hasBlocker(f, "servo status"));
  CHECK(hasBlocker(f, "multi-turn"));
  Fsm g(defaultConfig());
  g.step(ready());
  CHECK(hasBlocker(g, "not set"));
}

TEST(fsm_state_names) {
  CHECK(std::strcmp(stateName(State::PRE_TOUCH), "PRE_TOUCH") == 0);
  CHECK(std::strcmp(stateName(State::FAULT), "FAULT") == 0);
}

TEST(fsm_open_refused_on_servo_status_bits) {
  Fsm f(cfg());
  FsmInputs in = ready(); in.position = 5000; in.servoStatus = 0x20;
  MotorAction a = cmd(f, in, Cmd::OPEN);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(std::strstr(f.reason(), "Open refused: servo status") != nullptr);
}

TEST(fsm_open_refused_without_full_angle) {
  Fsm f(cfg());
  FsmInputs in = ready(); in.position = 5000; in.servoFullAngle = false;
  MotorAction a = cmd(f, in, Cmd::OPEN);
  CHECK(f.state() == State::IDLE);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(std::strstr(f.reason(), "multi-turn") != nullptr);
  CHECK(!f.openedSinceBoot());
}

TEST(fsm_failed_stop_retried_every_100ms) {
  Fsm f = preTouchFsm(100);
  FsmInputs in = ready(1000); in.position = 6000;
  f.motorFailed("timeout");
  CHECK(f.step(in).kind == MotorAction::STOP);   // first STOP right away
  f.motorFailed("timeout");                      // that STOP failed too
  for (uint32_t t = 1001; t < 1100; t++) { in.nowMs = t; CHECK(f.step(in).kind == MotorAction::NONE); }
  in.nowMs = 1100;
  CHECK(f.step(in).kind == MotorAction::STOP);   // retry after 100 ms
  in.nowMs = 1300;                               // that STOP succeeded: no more retries
  CHECK(f.step(in).kind == MotorAction::NONE);
  CHECK(f.state() == State::FAULT);
}

TEST(fsm_fault_reason_survives_later_reasons) {
  Fsm f = graspFsm(100);
  FsmInputs in = ready(120);
  in.anyFingerContact = true; in.position = 8000; in.maxForce = 3.1f;
  f.step(in);
  CHECK(std::strcmp(f.faultReason(), "FSR force limit") == 0);
  in.maxForce = 0;
  cmd(f, in, Cmd::OPEN);
  CHECK(std::strstr(f.reason(), "Reset first") != nullptr);
  cmd(f, in, Cmd::DISARM);
  cmd(f, in, Cmd::ESTOP);
  f.motorFailed("timeout");
  CHECK(std::strcmp(f.faultReason(), "FSR force limit") == 0);
  CHECK(f.faultFrom() == State::GRASP);
}

TEST(fsm_servo_not_in_mode0_blocks_and_refuses_open) {
  Fsm f(cfg());
  FsmInputs in = ready();
  in.servoMode = 1;
  f.step(in);
  Blocker b[Fsm::MAX_REPORTED];
  uint8_t n = f.blockers(b, Fsm::MAX_REPORTED);
  bool found = false;
  for (uint8_t i = 0; i < n; i++) if (strstr(b[i].text, "not position mode 0")) found = true;
  CHECK(found);
  in.cmd = Cmd::OPEN;
  MotorAction a = f.step(in);
  CHECK(a.kind == MotorAction::NONE);
  CHECK(f.state() == State::IDLE);
  CHECK(strstr(f.reason(), "not in position mode 0") != nullptr);
}

TEST(fsm_close_limit_reached_within_tolerance_faults) {
  Fsm f = openedFsm();
  FsmInputs in = ready();
  in.cmd = Cmd::ARM; f.step(in);
  in.cmd = Cmd::NONE; in.objectPresent = true;
  f.step(in); in.nowMs += START_STABLE_MS; f.step(in);   // IDLE -> START -> (300 ms) PRE_TOUCH
  CHECK(f.state() == State::PRE_TOUCH);
  in.position = cfg().closeLimit - 5;           // servo stopped just short of its goal
  in.nowMs += 10;
  f.step(in);
  CHECK(f.state() == State::FAULT);
  CHECK(strstr(f.faultReason(), "close limit") != nullptr);
}

TEST(fsm_not_homed_blocks_and_refuses_open) {
  Fsm f(cfg());
  FsmInputs in = ready();
  in.homed = false;
  f.step(in);
  Blocker b[Fsm::MAX_REPORTED];
  uint8_t n = f.blockers(b, Fsm::MAX_REPORTED);
  bool found = false;
  for (uint8_t i = 0; i < n; i++) if (strstr(b[i].text, "not homed")) found = true;
  CHECK(found);
  in.cmd = Cmd::OPEN;
  CHECK(f.step(in).kind == MotorAction::NONE);
  CHECK(strstr(f.reason(), "not homed") != nullptr);
}
TEST(fsm_homed_no_blocker) {
  Fsm f(cfg());
  FsmInputs in = ready();
  f.step(in);
  Blocker b[Fsm::MAX_REPORTED];
  uint8_t n = f.blockers(b, Fsm::MAX_REPORTED);
  for (uint8_t i = 0; i < n; i++) CHECK(strstr(b[i].text, "not homed") == nullptr);
}

// ---- phase 3: grasp plan ----
static Fsm startFsm(uint32_t t0) {
  Fsm f = openedFsm();
  cmd(f, ready(t0), Cmd::ARM);
  FsmInputs in = ready(t0); in.objectPresent = true;
  f.step(in);
  CHECK(f.state() == State::START);
  return f;
}
TEST(fsm_start_waits_for_stable_class) {
  Fsm f = startFsm(100);
  FsmInputs in = ready(100); in.objectPresent = true;
  for (uint32_t t = 100; t < 100 + START_TIMEOUT_MS; t += 100) {
    in.nowMs = t; in.plan.cls = ((t / 100) & 1) ? ObjClass::SMALL : ObjClass::MEDIUM;
    f.step(in);
    CHECK(f.state() == State::START);
  }
  in.nowMs = 100 + START_TIMEOUT_MS;
  f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "not stable") != nullptr);
}
TEST(fsm_start_aborts_without_measurement) {
  Fsm f = startFsm(100);
  FsmInputs in = ready(100); in.objectPresent = true;
  in.plan.valid = false; in.planWhy = "object not measured (no points in view)";
  in.nowMs = 100 + START_TIMEOUT_MS - 1; f.step(in);
  CHECK(f.state() == State::START);
  in.nowMs = 100 + START_TIMEOUT_MS; f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "object not measured") != nullptr);
}
TEST(fsm_start_refuses_force_above_max) {
  Fsm f = startFsm(100);                         // cfg(): maxForceN 3
  FsmInputs in = ready(100); in.objectPresent = true;
  in.plan.cls = ObjClass::LARGE; in.plan.prof.forceN = 4.0f;
  f.step(in);
  in.nowMs = 100 + START_STABLE_MS; f.step(in);
  CHECK(f.state() == State::IDLE);
  CHECK(!f.armed());
  CHECK(std::strstr(f.reason(), "Max force") != nullptr);
}
static Fsm approachFsm(uint32_t t0, int32_t goal, int32_t pos, bool approachOnly = false) {
  Fsm f = startFsm(t0);
  FsmInputs in = ready(t0); in.objectPresent = true; in.position = pos; in.approachOnly = approachOnly;
  in.plan = GraspPlan{true, ObjClass::SMALL, true, goal, ClassProfile{10, 1.0f, 200, 10}};
  f.step(in);
  in.nowMs = t0 + START_STABLE_MS;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  (void)a;
  return f;
}
TEST(fsm_pretouch_fast_then_slow) {
  Fsm f = startFsm(100);
  FsmInputs in = ready(100); in.objectPresent = true;
  in.plan = GraspPlan{true, ObjClass::SMALL, true, 9000, ClassProfile{10, 1.0f, 200, 10}};
  f.step(in);
  in.nowMs = 400;
  MotorAction a = f.step(in);
  CHECK(a.kind == MotorAction::MOVE);
  CHECK_EQ(a.position, 9000); CHECK_EQ(a.speed, 61); CHECK_EQ(a.torque, 200);
  in.nowMs = 450; in.position = 5000;
  CHECK(f.step(in).kind == MotorAction::NONE);
  in.nowMs = 500; in.position = 8995;
  a = f.step(in);
  CHECK(a.kind == MotorAction::RUN);
  CHECK_EQ(a.speed, 15); CHECK_EQ(a.torque, 200);
  CHECK(f.preTouchStep() == PreTouchStep::SLOW);
  CHECK(f.state() == State::PRE_TOUCH);
}
TEST(fsm_pretouch_goal_behind_skips_approach) {
  Fsm f = startFsm(100);
  FsmInputs in = ready(100); in.objectPresent = true; in.position = 9500;
  in.plan = GraspPlan{true, ObjClass::SMALL, true, 9000, ClassProfile{10, 1.0f, 200, 10}};
  f.step(in);
  in.nowMs = 400;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::PRE_TOUCH);
  CHECK(a.kind == MotorAction::RUN);
  CHECK_EQ(a.speed, 15);
  CHECK(f.preTouchStep() == PreTouchStep::SLOW);
}
TEST(fsm_contact_during_approach_goes_grasp) {
  Fsm f = approachFsm(100, 9000, 1968);
  FsmInputs in = ready(450); in.position = 4000; in.anyFingerContact = true;
  in.plan = GraspPlan{true, ObjClass::SMALL, true, 9000, ClassProfile{10, 1.0f, 200, 7}};   // live plan ignored: latched
  MotorAction a = f.step(in);
  CHECK(f.state() == State::GRASP);
  CHECK(a.kind == MotorAction::RUN);
  CHECK_EQ(a.speed, 10);
}
TEST(fsm_approach_stall_goes_slow) {
  Fsm f = approachFsm(100, 9000, 1968);
  FsmInputs in = ready(450); in.position = 8700;
  f.step(in);                                    // progress: 1968 -> 8700
  in.nowMs = 450 + APPROACH_STALL_MS - 1; f.step(in);
  CHECK(f.preTouchStep() == PreTouchStep::APPROACH);
  in.nowMs = 450 + APPROACH_STALL_MS;
  MotorAction a = f.step(in);
  CHECK(a.kind == MotorAction::RUN);
  CHECK_EQ(a.speed, 15);
  CHECK(f.preTouchStep() == PreTouchStep::SLOW);
}
TEST(fsm_approach_only_stops) {
  Fsm f = approachFsm(100, 9000, 1968, true);
  FsmInputs in = ready(450); in.position = 9000; in.approachOnly = true;
  MotorAction a = f.step(in);
  CHECK(f.state() == State::MONITOR);
  CHECK(a.kind == MotorAction::HOLD);
  CHECK(std::strstr(f.reason(), "approach only") != nullptr);
}
TEST(fsm_grasp_uses_class_force) {
  GripConfig c = cfg(); c.graspForceN = 5.0f;    // legacy field no longer used
  Fsm f = graspFsm(400);
  f.setConfig(c);
  FsmInputs in = ready(410);
  in.anyFingerContact = true; in.position = 8000;
  in.avgContactForce = 1.2f; in.channelsInContact = 2;
  for (uint32_t s = 101; s < 101 + 3; s++) { in.fsrSeq = s; in.nowMs++; f.step(in); }
  CHECK(f.state() == State::MONITOR);
}
TEST(fsm_delicate_grasp_speed) {
  Fsm f = startFsm(100);
  FsmInputs in = ready(100); in.objectPresent = true;
  in.plan = GraspPlan{true, ObjClass::DELICATE, false, 0, ClassProfile{15, 0.4f, 150, 3}};
  f.step(in);
  in.nowMs = 400;
  MotorAction a = f.step(in);
  CHECK(a.kind == MotorAction::RUN); CHECK_EQ(a.torque, 150);
  in.nowMs = 410; in.anyFingerContact = true; in.position = 6000;
  a = f.step(in);
  CHECK(f.state() == State::GRASP);
  CHECK_EQ(a.speed, 3); CHECK_EQ(a.torque, 150);
}
