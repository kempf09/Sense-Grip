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
