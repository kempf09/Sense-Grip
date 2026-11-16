#include "check.h"
#include "config_core.h"

TEST(config_defaults_in_range) {
  auto c = defaultConfig();
  CHECK(!clampConfig(c));
}
TEST(config_clamps_speed) {
  auto c = defaultConfig();
  c.approachSpeed = 2000;                      // value saved by an earlier build
  CHECK(clampConfig(c));
  CHECK_EQ(c.approachSpeed, (uint16_t)61);
}
TEST(config_limits_problem) {
  auto c = defaultConfig();
  CHECK(limitsProblem(c) != nullptr);
  c.openZero = 1968; c.closeLimit = 19102; c.closePositive = true; c.limitsValid = true;
  CHECK(limitsProblem(c) == nullptr);
  c.closePositive = false;
  CHECK(limitsProblem(c) != nullptr);
}

TEST(config_default_speeds_in_servo_units) {
  auto c = defaultConfig();
  CHECK_EQ(c.approachSpeed, (uint16_t)61);
  CHECK_EQ(c.graspSpeed, (uint16_t)10);
  CHECK_EQ(c.openSpeed, (uint16_t)61);
}
TEST(config_fsr_enabled_default_all_and_clamped) {
  auto c = defaultConfig();
  CHECK_EQ(c.fsrEnabled, (uint8_t)0x3F);
  c.fsrEnabled = 0xFF;
  CHECK(clampConfig(c));
  CHECK_EQ(c.fsrEnabled, (uint8_t)0x3F);
}
TEST(config_v2_blob_is_v3_without_fsr_enabled) {
  CHECK_EQ((unsigned)GRIP_CONFIG_V2_SIZE, (unsigned)__builtin_offsetof(GripConfig, tofMask) + 24u);
}

#include <cstddef>
#include <cmath>
TEST(config_filter_defaults) {
  auto c = defaultConfig();
  CHECK_EQ(c.fsrFilter, (uint8_t)2);
  CHECK_EQ(c.fsrFilterWindow, (uint8_t)5);
  CHECK_NEAR(c.fsrFilterAlpha, 0.3f, 1e-6);
  CHECK_NEAR(c.fsrKalmanQ, 0.01f, 1e-7);
  CHECK_NEAR(c.fsrKalmanR, 0.05f, 1e-7);
  CHECK(!clampConfig(c));
}
TEST(config_filter_fields_clamped) {
  auto c = defaultConfig();
  c.fsrFilter = 9; c.fsrFilterWindow = 50; c.fsrFilterAlpha = 0; c.fsrKalmanQ = NAN;
  CHECK(clampConfig(c));
  CHECK_EQ(c.fsrFilter, (uint8_t)2);
  CHECK_EQ(c.fsrFilterWindow, (uint8_t)20);
  CHECK_NEAR(c.fsrFilterAlpha, 0.05f, 1e-6);
  CHECK_NEAR(c.fsrKalmanQ, 0.01f, 1e-7);
  c.fsrFilterWindow = 0;
  CHECK(clampConfig(c));
  CHECK_EQ(c.fsrFilterWindow, (uint8_t)2);
}
TEST(config_blob_sizes_ordered) {
  CHECK_EQ((unsigned)GRIP_CONFIG_V2_SIZE, (unsigned)offsetof(GripConfig, fsrEnabled));
  CHECK(GRIP_CONFIG_V4_SIZE > GRIP_CONFIG_V2_SIZE);
  CHECK(GRIP_CONFIG_V4_SIZE < sizeof(GripConfig));
}
TEST(config_filter_params_of) {
  FilterParams p = filterParamsOf(defaultConfig());
  CHECK(p.kind == FilterKind::MEDIAN);
  CHECK_EQ(p.window, (uint8_t)5);
}
TEST(config_v4_blob_size_includes_trailing_padding) {
  // v3/v4 builds saved sizeof(their GripConfig): fields up to fsrEnabled, padded to 8.
  CHECK_EQ((unsigned)GRIP_CONFIG_V4_BLOB_SIZE, (unsigned)(GRIP_CONFIG_V2_SIZE + 8));
  CHECK_EQ((unsigned)GRIP_CONFIG_V4_SIZE, (unsigned)(GRIP_CONFIG_V2_SIZE + 1));
}
TEST(config_touch_defaults) {
  auto c = defaultConfig();
  CHECK_EQ(c.touchRef, 0);
  CHECK(!c.touchValid);
}
TEST(config_v5_blob_sizes) {
  CHECK_EQ((unsigned)GRIP_CONFIG_V5_SIZE, (unsigned)offsetof(GripConfig, touchRef));
  CHECK_EQ((unsigned)(GRIP_CONFIG_V5_BLOB_SIZE % alignof(GripConfig)), 0u);
  CHECK(GRIP_CONFIG_V5_BLOB_SIZE >= GRIP_CONFIG_V5_SIZE);
}

TEST(limits_restorable_after_home) {
  GripConfig c = defaultConfig();
  c.openZero = 1000; c.closeLimit = 20000; c.closePositive = true; c.limitsValid = false;
  CHECK(!limitsRestorable(c));                         // no touch reference: unknown frame
  c.touchValid = true; c.touchRef = 20100;
  CHECK(limitsRestorable(c));
  c.closeLimit = 500;                                  // not on the closing side
  CHECK(!limitsRestorable(c));
  c = defaultConfig(); c.touchValid = true;            // never set (both 0)
  CHECK(!limitsRestorable(c));
}

// ---- v7: class thresholds and grasp profiles (phase 3) ----
#include <cstring>
static bool sameProfile(ClassProfile p, float st, float f, uint16_t cur, uint16_t sp) {
  return std::fabs(p.standoffMm - st) < 1e-4f && std::fabs(p.forceN - f) < 1e-4f && p.maxCurrent == cur && p.graspSpeed == sp;
}
TEST(config_v7_defaults) {
  GripConfig c = defaultConfig();
  CHECK_EQ(c.smallMaxMm, (uint16_t)50);
  CHECK_EQ(c.mediumMaxMm, (uint16_t)90);
  CHECK_EQ(c.contactSpeed, (uint16_t)15);
  CHECK_NEAR(c.maxForceN, 8.0f, 1e-6);
  CHECK(sameProfile(profileOf(c, ObjClass::SMALL), 10, 1.0f, 200, 5));
  CHECK(sameProfile(profileOf(c, ObjClass::MEDIUM), 10, 2.0f, 260, 5));
  CHECK(sameProfile(profileOf(c, ObjClass::LARGE), 10, 4.0f, 350, 5));
  CHECK(sameProfile(profileOf(c, ObjClass::DELICATE), 15, 0.4f, 150, 3));
  CHECK(!clampConfig(c));
  CHECK(profileProblem(c) == nullptr);
}
TEST(config_profile_problem) {
  GripConfig c = defaultConfig();
  c.largeForceN = 9;
  CHECK(profileProblem(c) != nullptr);
  c = defaultConfig(); c.delicateForceN = 0.2f;              // below contact force 0.25
  CHECK(profileProblem(c) != nullptr);
  c = defaultConfig(); c.smallMaxMm = 95; c.mediumMaxMm = 90;
  CHECK(profileProblem(c) != nullptr);
}
TEST(config_v6_migrated_max_force) {
  GripConfig c = defaultConfig(); c.maxForceN = 3.0f;      // a stored v6 config keeps 3 N
  const char* p = profileProblem(c);
  CHECK(p != nullptr);
  CHECK(std::strstr(p, "large") != nullptr);
  CHECK(std::strstr(p, "Max force") != nullptr);
}
TEST(config_v6_size_constants) {
  CHECK_EQ((unsigned)GRIP_CONFIG_V6_SIZE, (unsigned)offsetof(GripConfig, smallMaxMm));
  CHECK(GRIP_CONFIG_V6_BLOB_SIZE >= GRIP_CONFIG_V6_SIZE);
  CHECK_EQ((unsigned)(GRIP_CONFIG_V6_BLOB_SIZE % alignof(GripConfig)), 0u);
}
TEST(config_v7_fields_clamped) {
  GripConfig c = defaultConfig();
  c.largeMaxCurrent = 999; c.smallGraspSpeed = 0; c.mediumStandoffMm = 80; c.contactSpeed = 200;
  CHECK(clampConfig(c));
  CHECK_EQ(c.largeMaxCurrent, (uint16_t)415);
  CHECK_EQ(c.smallGraspSpeed, (uint16_t)1);
  CHECK_NEAR(c.mediumStandoffMm, 50.0f, 1e-6);
  CHECK_EQ(c.contactSpeed, SPEED_MAX);
}

// ---- v8: sweep settings ----
TEST(config_v8_sweep_defaults) {
  GripConfig c = defaultConfig();
  CHECK_EQ(c.sweepStepCounts, (uint16_t)100);
  CHECK_EQ(c.sweepSpeed, (uint16_t)5);
  CHECK_EQ(c.sweepCurrent, (uint16_t)0);                  // 0 = use Close max current
  CHECK(!clampConfig(c));
}
TEST(config_v8_sweep_clamped) {
  GripConfig c = defaultConfig();
  c.sweepStepCounts = 10; c.sweepSpeed = 99; c.sweepCurrent = 999;
  CHECK(clampConfig(c));
  CHECK_EQ(c.sweepStepCounts, (uint16_t)60);              // SWEEP_MAX samples must cover ~13 000 counts
  CHECK_EQ(c.sweepSpeed, (uint16_t)20);
  CHECK_EQ(c.sweepCurrent, (uint16_t)415);
  c.sweepCurrent = 20; clampConfig(c);
  CHECK_EQ(c.sweepCurrent, (uint16_t)50);
  c.sweepCurrent = 0; CHECK(!clampConfig(c));
}
TEST(config_v7_size_constants) {
  CHECK_EQ((unsigned)GRIP_CONFIG_V7_SIZE, (unsigned)offsetof(GripConfig, sweepStepCounts));
  CHECK_EQ((unsigned)GRIP_CONFIG_V7_BLOB_SIZE, 168u);      // sizeof(GripConfig) of the v7 build (measured)
}
