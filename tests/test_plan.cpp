#include "check.h"
#include "plan_core.h"
#include <cmath>
#include <cstring>
#include <vector>

// Synthetic sweep with a known opening: 100 samples from openZero 0, +100 counts each; opening
// falls linearly from 100 mm (sample 0) to 1 mm (sample 99). Built directly (no frames needed).
static std::vector<SweepSample> S(100);
static SweepModel M;
static void build() {
  M.n = 100; M.openZero = 0; M.touchRef = 9900; M.firstSeen = 0;
  for (int i = 0; i < 100; i++) { S[i].pos = 100 * i; S[i].valid = 7; M.opening[i] = 100.0f - i; }
}
static GripConfig cfg() {
  GripConfig c = defaultConfig();
  c.openZero = 0; c.closeLimit = 9800; c.closePositive = true; c.limitsValid = true;
  return c;
}
static ObjectMeasure obj(float width, float range) {
  ObjectMeasure m{}; m.valid = true; m.widthMm = width; m.rangeMm = range; return m;
}

TEST(plan_auto_measured) {
  build();
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), obj(60, 100), Override::AUTO, false, 0);
  CHECK(r.plan.valid);
  CHECK(r.detectedValid);
  CHECK(r.detected == ObjClass::MEDIUM);
  CHECK(r.plan.cls == ObjClass::MEDIUM);
  CHECK(r.plan.approach);
  // stand-off opening = 30 + 10 + 0.1 * 100 = 50 mm
  CHECK(std::fabs(openingAt(M, S.data(), r.plan.approachGoal) - 50.0f) <= 1.0f);
  CHECK_NEAR(r.plan.prof.forceN, 2.0f, 1e-6);
}
TEST(plan_override_without_measure) {
  build();
  ObjectMeasure none{};
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), none, Override::LARGE, false, 0);
  CHECK(r.plan.valid);
  CHECK(r.plan.cls == ObjClass::LARGE);
  CHECK(!r.plan.approach);
  CHECK(!r.detectedValid);
}
TEST(plan_override_with_measure_keeps_approach) {
  build();
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), obj(44, 100), Override::LARGE, false, 0);
  CHECK(r.plan.cls == ObjClass::LARGE);
  CHECK(r.detected == ObjClass::SMALL);
  CHECK(r.plan.approach);
}
TEST(plan_delicate_beats_override) {
  build();
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), obj(60, 100), Override::LARGE, true, 0);
  CHECK(r.plan.cls == ObjClass::DELICATE);
  CHECK_NEAR(r.plan.prof.standoffMm, 15.0f, 1e-6);
  CHECK_NEAR(r.plan.prof.forceN, 0.4f, 1e-6);
  CHECK_EQ(r.plan.prof.maxCurrent, (uint16_t)150);
  CHECK_EQ(r.plan.prof.graspSpeed, (uint16_t)3);
}
TEST(plan_delicate_alone_is_valid_without_measure) {
  build();
  ObjectMeasure none{};
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), none, Override::AUTO, true, 0);
  CHECK(r.plan.valid);
  CHECK(!r.plan.approach);
}
TEST(plan_invalid_reasons) {
  build();
  ObjectMeasure none{};
  CHECK(std::strstr(buildPlan(cfg(), false, &M, S.data(), obj(60, 100), Override::AUTO, false, 0).why, "not homed") != nullptr);
  CHECK(std::strstr(buildPlan(cfg(), true, nullptr, nullptr, obj(60, 100), Override::AUTO, false, 0).why, "no sweep") != nullptr);
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), none, Override::AUTO, false, 0);
  CHECK(!r.plan.valid);
  CHECK(std::strstr(r.why, "object not measured") != nullptr);
}
TEST(plan_goal_never_past_close_limit) {
  build();
  GripConfig c = cfg(); c.smallStandoffMm = 0;
  // opening target 0.5 + 0 + 0.1 mm is never reached (table ends at 1 mm, position 9900)
  PlanResult r = buildPlan(c, true, &M, S.data(), obj(1, 1), Override::AUTO, false, 0);
  CHECK_EQ(r.plan.approachGoal, 9800);
}
TEST(plan_goal_behind_returns_position) {
  build();
  // Present opening at 5000 is 50 mm; a 120 mm object needs 60 + 10 + 10 = 80 mm: already closer.
  PlanResult r = buildPlan(cfg(), true, &M, S.data(), obj(120, 100), Override::AUTO, false, 5000);
  CHECK_EQ(r.plan.approachGoal, 5000);
}
