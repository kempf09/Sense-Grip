#include "check.h"
#include "tof_detect.h"

static void fill(uint16_t* mm, uint16_t v) { for (int i = 0; i < 64; i++) mm[i] = v; }

TEST(tof_min_ignores_invalid) {
  uint16_t mm[64];
  fill(mm, 0);
  CHECK_EQ(frameMinMm(mm, ~0ULL), (uint16_t)0xFFFF);
  mm[1] = 4000; mm[2] = 5000;
  CHECK_EQ(frameMinMm(mm, ~0ULL), (uint16_t)0xFFFF);
  mm[3] = 3999; mm[4] = 700;
  CHECK_EQ(frameMinMm(mm, ~0ULL), (uint16_t)700);
  mm[7] = 10;
  CHECK_EQ(frameMinMm(mm, ~0ULL), (uint16_t)10);
  CHECK_EQ(frameMinMm(mm, ~(1ULL << 7)), (uint16_t)700);
  mm[63] = 5;
  CHECK_EQ(frameMinMm(mm, ~(1ULL << 7)), (uint16_t)5);
  CHECK_EQ(frameMinMm(mm, ~(1ULL << 7) & ~(1ULL << 63)), (uint16_t)700);
  CHECK_EQ(frameMinMm(mm, 0), (uint16_t)0xFFFF);
}

TEST(tof_presence_needs_hold) {
  PresenceTimer t{};
  uint16_t m[3] = {50, 0xFFFF, 0xFFFF};
  uint32_t age[3] = {0, 0, 0};
  CHECK(!updatePresence(t, m, age, 80, 200, 0));
  CHECK(!updatePresence(t, m, age, 80, 200, 199));
  CHECK(updatePresence(t, m, age, 80, 200, 200));
}

TEST(tof_presence_resets_on_gap) {
  PresenceTimer t{};
  uint16_t m[3] = {50, 0xFFFF, 0xFFFF};
  uint32_t age[3] = {0, 0, 0};
  CHECK(!updatePresence(t, m, age, 80, 200, 0));
  m[0] = 120;
  CHECK(!updatePresence(t, m, age, 80, 200, 150));
  CHECK(!t.active);
  m[0] = 50;
  CHECK(!updatePresence(t, m, age, 80, 200, 200));
  CHECK(!updatePresence(t, m, age, 80, 200, 399));
  CHECK(updatePresence(t, m, age, 80, 200, 400));
}

TEST(tof_any_one_sensor_triggers) {
  PresenceTimer t{};
  uint16_t m[3] = {0xFFFF, 0xFFFF, 60};
  uint32_t age[3] = {0, 0, 0};
  CHECK(!updatePresence(t, m, age, 80, 100, 10));
  CHECK(updatePresence(t, m, age, 80, 100, 110));
  PresenceTimer z{};
  CHECK(updatePresence(z, m, age, 80, 0, 5));
}

TEST(tof_stale_frame_not_present) {
  PresenceTimer t{};
  uint16_t m[3] = {50, 0xFFFF, 0xFFFF};
  uint32_t age[3] = {0, 0, 0};
  CHECK(!updatePresence(t, m, age, 80, 200, 0));
  age[0] = 600;
  CHECK(!updatePresence(t, m, age, 80, 200, 300));
  CHECK(!t.active);
  age[0] = 0;
  CHECK(!updatePresence(t, m, age, 80, 200, 350));
  CHECK(updatePresence(t, m, age, 80, 200, 550));
}

TEST(tof_presence_wrap_safe) {
  PresenceTimer t{};
  uint16_t m[3] = {50, 0xFFFF, 0xFFFF};
  uint32_t age[3] = {0, 0, 0};
  CHECK(!updatePresence(t, m, age, 80, 200, 0xFFFFFFF0u));
  CHECK(updatePresence(t, m, age, 80, 200, 0xFFFFFFF0u + 200u));
}
