#include "check.h"
#include "sweep_core.h"
#include <cmath>
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

// Hardware sweep 2026-11-16 (/api/sweep, firstSeen 12): measured fingertip opening, raw from the
// analysis before smoothing existed. Single-sample spikes: 167.1 / 156.4 mm, dips 30.5 / 27.6 mm.
static const float HW_OPENING[] = {99.8, 90.0, 87.1, 167.1, 81.2, 80.3, 86.1, 79.3, 156.4, 80.3, 73.5, 77.3, 75.4, 76.4, 75.4, 74.4, 73.5, 68.6, 69.6, 66.7, 63.8, 67.7, 73.5, 62.8, 67.7, 67.7, 68.6, 68.6, 60.0, 62.8, 63.8, 64.8, 63.8, 64.8, 61.9, 59.0, 59.0, 62.8, 54.2, 60.9, 54.2, 59.0, 53.2, 52.2, 51.2, 51.2, 49.2, 49.2, 49.2, 48.2, 47.2, 48.2, 45.3, 46.3, 44.3, 45.3, 44.3, 44.3, 42.3, 30.5, 42.3, 41.3, 41.3, 41.3, 27.6, 28.5, 26.6, 29.5, 27.6, 26.6, 26.6, 27.6, 23.7, 23.7, 25.6, 23.7, 22.7, 24.6, 21.7, 23.7, 23.7, 22.7, 22.7, 21.7, 22.7, 21.7, 19.8, 20.8, 22.7, 21.7, 20.8, 21.7, 21.7, 21.7, 21.7, 20.8, 20.8, 19.8, 20.8, 17.9, 18.9, 19.8, 17.9, 18.9, 19.8, 17.0, 17.0, 17.0, 16.1, 17.0, 16.1, 15.1, 17.0, 10.9, 11.5, 12.1, 14.2, 12.1, 13.4, 13.3, 12.1, 14.1, 12.1, 12.4, 12.4, 12.4, 13.3, 10.7, 10.1, 11.4, 12.4, 13.3, 12.1, 11.6, 12.1, 10.8};
TEST(sweep_smoothing_hw_monotonic_without_spikes) {
  const int n = sizeof(HW_OPENING) / sizeof(HW_OPENING[0]);
  std::vector<float> o(HW_OPENING, HW_OPENING + n);
  smoothOpening(o.data(), n);
  for (int i = 1; i < n; i++) CHECK(o[i] <= o[i - 1] + 1e-4f);
  CHECK(o[0] <= 100.0f);                                  // spike 167.1 at raw 15 gone
  CHECK(o[3] < 100.0f && o[8] < 100.0f);
  CHECK(o[71 - 12] >= 40.0f);                             // dip 30.5 between ~42 mm neighbours gone
  CHECK(o[n - 1] >= 9.0f && o[n - 1] <= 14.0f);           // touch ~11-12 mm kept
}
TEST(sweep_smoothing_keeps_short_or_clean) {
  float a[3] = {50, 40, 30};
  smoothOpening(a, 3);
  CHECK_NEAR(a[0], 50, 1e-4); CHECK_NEAR(a[2], 30, 1e-4);
}
TEST(sweep_analysis_removes_single_sample_spike) {
  auto v = makeSweep();
  for (int s = 0; s < 3; s++) for (int z : {10, 11, 18, 19}) v[60].mm[s][z] += 40;   // one wild frame
  SweepModel m = analyze(v);
  CHECK(m.opening[60] <= m.opening[59] + 1e-3f);
  CHECK(m.opening[60] >= m.opening[61] - 1e-3f);
  CHECK(std::fabs(m.opening[60] - 0.5f * (m.opening[59] + m.opening[61])) < 2.0f);
}
