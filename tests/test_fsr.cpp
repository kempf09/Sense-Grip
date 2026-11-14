#include "check.h"
#include "fsr_math.h"

TEST(fsr_ohms_divider) {
  CHECK_NEAR(fsrOhms(1650, 2000), 2000, 1);
  CHECK_EQ(fsrOhms(0.5f, 2000), 10e6f);
  CHECK_EQ(fsrOhms(3300, 2000), 0.0f);
}
TEST(fsr_force_hits_cal_points) {
  FsrCal c{2000, 1, 30000, 10, 3000, 0};
  CHECK(fsrCalValid(c));
  CHECK_NEAR(fsrForce(c, 30000), 1, 0.01);
  CHECK_NEAR(fsrForce(c, 3000), 10, 0.01);
  CHECK_EQ(fsrForce(c, 0), 0.0f);
  FsrCal bad{2000, 1, 30000, 1, 3000, 0};
  CHECK(!fsrCalValid(bad));
  CHECK(std::isnan(fsrForce(bad, 5000)));
}
TEST(fsr_contact_debounce_needs_n_samples) {
  ContactDebounce d{};
  float f[6] = {0, 0, 0, 0, 0, 0.3f};
  f[5] = 0.3f;
  updateContacts(d, f, 0.25f, 3);
  updateContacts(d, f, 0.25f, 3);
  CHECK(!d.stable[0]);
  updateContacts(d, f, 0.25f, 3);
  CHECK(d.stable[0]);
  CHECK(!d.stable[1]);
  f[5] = 0.1f;
  updateContacts(d, f, 0.25f, 3);
  CHECK(!d.stable[0]);
  CHECK_EQ((int)d.count[0], 0);
}
TEST(fsr_aggregate_counts_contact_channels) {
  float f[6] = {0.3f, 0, 0, 0, 1.0f, 0};
  auto a = aggregate(f, 0.25f);
  CHECK_EQ((int)a.channelsInContact, 2);
  CHECK_NEAR(a.avgContactForce, 0.65, 1e-5);
  CHECK_NEAR(a.maxForce, 1.0, 1e-5);
  CHECK_NEAR(a.fingerForce[2], 1.0, 1e-5);
  CHECK_NEAR(a.fingerForce[0], 0.3, 1e-5);
}
