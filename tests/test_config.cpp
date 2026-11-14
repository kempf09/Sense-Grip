#include "check.h"
#include "config_core.h"

TEST(config_defaults_in_range) {
  auto c = defaultConfig();
  CHECK(!clampConfig(c));
}
TEST(config_clamps_speed) {
  auto c = defaultConfig();
  c.approachSpeed = 5000;
  CHECK(clampConfig(c));
  CHECK_EQ(c.approachSpeed, (uint16_t)SPEED_MAX);
  c.approachSpeed = 500;                       // speeds above the old 61 cap are valid now
  CHECK(!clampConfig(c));
  CHECK_EQ(c.approachSpeed, (uint16_t)500);
}
TEST(config_limits_problem) {
  auto c = defaultConfig();
  CHECK(limitsProblem(c) != nullptr);
  c.openZero = 1968; c.closeLimit = 19102; c.closePositive = true; c.limitsValid = true;
  CHECK(limitsProblem(c) == nullptr);
  c.closePositive = false;
  CHECK(limitsProblem(c) != nullptr);
}

TEST(config_default_speeds_are_steps_per_second) {
  auto c = defaultConfig();
  CHECK_EQ(c.approachSpeed, (uint16_t)2000);
  CHECK_EQ(c.graspSpeed, (uint16_t)300);
  CHECK_EQ(c.openSpeed, (uint16_t)2000);
}
TEST(config_upgrade_from_v1_replaces_speeds_keeps_limits) {
  auto c = defaultConfig();
  c.approachSpeed = 61; c.graspSpeed = 10; c.openSpeed = 61;
  c.openZero = 1968; c.closeLimit = 19102; c.limitsValid = true; c.maxForceN = 2.5f;
  upgradeFromV1(c);
  CHECK_EQ(c.approachSpeed, (uint16_t)2000);
  CHECK_EQ(c.graspSpeed, (uint16_t)300);
  CHECK_EQ(c.openSpeed, (uint16_t)2000);
  CHECK_EQ(c.openZero, 1968);
  CHECK_EQ(c.closeLimit, 19102);
  CHECK(c.limitsValid);
  CHECK_NEAR(c.maxForceN, 2.5f, 1e-6);
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
