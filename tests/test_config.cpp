#include "check.h"
#include "config_core.h"

TEST(config_defaults_in_range) {
  auto c = defaultConfig();
  CHECK(!clampConfig(c));
}
TEST(config_clamps_speed) {
  auto c = defaultConfig();
  c.approachSpeed = 500;
  CHECK(clampConfig(c));
  CHECK_EQ(c.approachSpeed, 61);
}
TEST(config_limits_problem) {
  auto c = defaultConfig();
  CHECK(limitsProblem(c) != nullptr);
  c.openZero = 1968; c.closeLimit = 19102; c.closePositive = true; c.limitsValid = true;
  CHECK(limitsProblem(c) == nullptr);
  c.closePositive = false;
  CHECK(limitsProblem(c) != nullptr);
}
