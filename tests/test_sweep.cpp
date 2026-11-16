#include "check.h"
#include "sweep_core.h"
#include <cstring>
#include <vector>

// Synthetic sweep: 200 samples from openZero 2000, +100 counts each. Finger zones 10,11,18,19 of
// every sensor read 30 + 0.35*i mm (finger moving inward along the ray), invisible (0) below
// pos 3000; every other zone reads 1200 (background).
static const int N = 200;
static std::vector<SweepSample> makeSweep() {
  std::vector<SweepSample> v(N);
  for (int i = 0; i < N; i++) {
    v[i].pos = 2000 + 100 * i; v[i].valid = 0x7;
    for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) {
      const bool finger = z == 10 || z == 11 || z == 18 || z == 19;
      v[i].mm[s][z] = finger ? (v[i].pos < 3000 ? 0 : (uint16_t)(30 + 0.35f * i)) : 1200;
    }
  }
  return v;
}
static SweepModel analyze(const std::vector<SweepSample>& v) {
  SensorCalib cal[3] = {defaultCalib(), defaultCalib(), defaultCalib()};
  static SweepModel m;
  sweepAnalyze(v.data(), N, cal, 2000, v[N - 1].pos, true, m);
  return m;
}

TEST(sweep_finger_zones_found) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  const uint64_t want = (1ULL << 10) | (1ULL << 11) | (1ULL << 18) | (1ULL << 19);
  for (int s = 0; s < 3; s++) CHECK(m.fingerZones[s] == want);
}
TEST(sweep_opening_monotonic) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  CHECK_NEAR(m.opening[0], 170.28f, 0.01);
  for (int i = 1; i < N; i++) CHECK(m.opening[i] <= m.opening[i - 1] + 1e-3f);
  CHECK_NEAR(openingAt(m, v.data(), 2000), 170.28f, 0.01);
}
TEST(sweep_estimate_recovers_position) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  uint16_t frame[3][64];
  std::memcpy(frame, v[120].mm, sizeof(frame));
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) if (frame[s][z]) frame[s][z] += (z & 1) ? 3 : -3;
  PosEstimate e = estimatePosition(m, v.data(), frame, 0x7);
  CHECK(e.valid);
  // Synthetic readings change 0.35 mm/sample and are whole mm: resolution is ~3 samples.
  CHECK(e.pos >= v[120].pos - 300 && e.pos <= v[120].pos + 300);
}
TEST(estimate_invalid_without_finger_zones) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  uint16_t frame[3][64];
  std::memcpy(frame, v[120].mm, sizeof(frame));
  for (int s = 0; s < 3; s++) { frame[s][10] = frame[s][11] = frame[s][18] = frame[s][19] = 0; }
  CHECK(!estimatePosition(m, v.data(), frame, 0x7).valid);
}
TEST(check_detects_mismatch) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  CHECK(positionMismatch(m, v.data(), v[40].pos, v[150].mm, 0x7, 25.0f));
  CHECK(!positionMismatch(m, v.data(), v[150].pos, v[150].mm, 0x7, 25.0f));
}
TEST(check_suspended_when_object_present) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  uint16_t frame[3][64];
  std::memcpy(frame, v[150].mm, sizeof(frame));
  for (int s = 0; s < 3; s++) { frame[s][10] = frame[s][11] = frame[s][18] = frame[s][19] = 0; }
  CHECK(!positionMismatch(m, v.data(), v[40].pos, frame, 0x7, 25.0f));
}
TEST(sweep_rejects_non_monotonic_or_short) {
  auto v = makeSweep();
  CHECK(sweepValid(v.data(), N, 2000, true));
  CHECK(!sweepValid(v.data(), 19, 2000, true));
  auto b = v; b[50].pos = b[49].pos - 10;
  CHECK(!sweepValid(b.data(), N, 2000, true));
  CHECK(!sweepValid(v.data(), N, 1600, true));       // first sample 400 counts from openZero
}

TEST(view_obstructed_by_something_besides_fingers) {
  auto v = makeSweep(); SweepModel m = analyze(v);
  uint16_t frame[3][64];
  std::memcpy(frame, v[150].mm, sizeof(frame));
  CHECK(!viewObstructed(m, v.data(), v[150].pos, frame, 0x7));
  for (int z : {30, 31, 38}) frame[1][z] = 300;        // something at 300 mm in background zones
  CHECK(viewObstructed(m, v.data(), v[150].pos, frame, 0x7));
}

// Hardware: near OPEN the finger zones read the background (not 0), so the plot started at the
// background "radius" (~80 mm) instead of 170 mm. Readings that equal the open background, or
// fall inside the invisible region, must not count as fingertip points.
TEST(sweep_opening_ignores_background_in_finger_zones) {
  auto v = makeSweep();
  for (int i = 0; i < N; i++)
    for (int s = 0; s < 3; s++) for (int z : {10, 11, 18, 19})
      if (v[i].pos < 4000) v[i].mm[s][z] = 400;          // background until the finger enters at 4000
      else v[i].mm[s][z] = (uint16_t)(400 - 1.5f * (i - 20)); // finger enters, moves along the ray
  SweepModel m = analyze(v);
  CHECK_NEAR(m.opening[0], 170.28f, 0.01);
  CHECK(m.opening[5] > m.opening[25]);                   // extrapolated part above the first seen point
  for (int i = 1; i < 100; i++) CHECK(m.opening[i] <= m.opening[i - 1] + 1e-3f);
}

// SEN0628 range error is about 10 %: a far zone may differ from the sweep by more than 25 mm
// without the position being wrong.
TEST(check_tolerates_ten_percent_at_long_range) {
  auto v = makeSweep();
  for (int i = 0; i < N; i++) for (int s = 0; s < 3; s++) for (int z : {10, 11, 18, 19})
    v[i].mm[s][z] = v[i].pos < 3000 ? 0 : (uint16_t)(600 - 1.0f * i);
  SweepModel m = analyze(v);
  uint16_t frame[3][64];
  std::memcpy(frame, v[150].mm, sizeof(frame));
  for (int s = 0; s < 3; s++) for (int z : {10, 11, 18, 19}) frame[s][z] = (uint16_t)(frame[s][z] * 1.08f);
  CHECK(!positionMismatch(m, v.data(), v[150].pos, frame, 0x7, 25.0f));
  CHECK(positionMismatch(m, v.data(), v[40].pos, v[150].mm, 0x7, 25.0f));
}

TEST(sweep_invalid_why_names_the_problem) {
  auto v = makeSweep();
  char b[64];
  CHECK(sweepInvalidWhy(v.data(), N, 2000, true, b, sizeof b) == nullptr);
  CHECK(std::strstr(sweepInvalidWhy(v.data(), 10, 2000, true, b, sizeof b), "10 samples") != nullptr);
  CHECK(std::strstr(sweepInvalidWhy(v.data(), N, 1600, true, b, sizeof b), "400 counts") != nullptr);
  auto w = v; w[50].pos = w[49].pos;
  CHECK(std::strstr(sweepInvalidWhy(w.data(), N, 2000, true, b, sizeof b), "sample 50") != nullptr);
}
