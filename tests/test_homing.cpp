#include "check.h"
#include "homing_core.h"

TEST(home_resolves_exact_turns) {
  HomeResult r = homeResolve(19202 - 2 * 4096, 19202, 600);
  CHECK(r.ok); CHECK_EQ(r.posOffset, 8192); CHECK_EQ(r.residual, 0);
}
TEST(home_resolves_with_small_residual) {
  HomeResult r = homeResolve(19202 - 4096 + 150, 19202, 600);
  CHECK(r.ok); CHECK_EQ(r.posOffset, 4096); CHECK_EQ(r.residual, 150);
}
TEST(home_rejects_large_residual) {
  HomeResult r = homeResolve(19202 + 1500 - 4096, 19202, 600);
  CHECK(!r.ok); CHECK(r.why != nullptr);
  CHECK(r.residual == 1500 || r.residual == 1500 - 4096);
}
TEST(home_resolves_when_already_at_touch) {
  HomeResult r = homeResolve(19202, 19202, 600);
  CHECK(r.ok); CHECK_EQ(r.posOffset, 0); CHECK_EQ(r.residual, 0);
}
TEST(home_negative_positions) {
  HomeResult r = homeResolve(-3000 + 3 * 4096, -3000, 600);
  CHECK(r.ok); CHECK_EQ(r.posOffset, -12288);
}
TEST(turn_of) {
  CHECK_EQ(turnOf(19202 - 4096 * 2 - 10, 19202), -3);
  CHECK_EQ(turnOf(19202, 19202), 0);
  CHECK_EQ(turnOf(19202 + 4095, 19202), 0);
}
TEST(stall_rule) {
  CHECK(stalled(1009, 1000, 300));
  CHECK(!stalled(1009, 1000, 299));
  CHECK(!stalled(1010, 1000, 400));
}
TEST(unstick_only_past_open) {
  // closePositive: opening side = smaller positions; openZero 2000.
  CHECK(unstickAllowed(true, 1900, 2000, true, true, false));    // homed, past open
  CHECK(!unstickAllowed(true, 9000, 2000, true, true, false));   // homed, mid-travel
  CHECK(!unstickAllowed(false, 0, 2000, true, true, false));     // not homed, fingers visible
  CHECK(!unstickAllowed(false, 0, 2000, true, false, true));     // not homed, invisible, contact
  CHECK(unstickAllowed(false, 0, 2000, true, false, false));     // not homed, invisible, no contact
  CHECK(!unstickAllowed(true, 1900, 2000, true, true, true));    // contact always forbids
}
