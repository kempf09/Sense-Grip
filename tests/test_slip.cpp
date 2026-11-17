#include "check.h"
#include "slip_core.h"
#include <cmath>
#include <cstring>

namespace {
uint16_t FR[3][64];
SlipInput in(uint32_t t, float f, bool active = true) {
  SlipInput x{}; x.tMs = t; x.active = active;
  for (int i = 0; i < 6; i++) x.force[i] = (i < 2) ? f : 0.0f;   // two pads loaded
  x.frame = FR; x.frameValid = 0x7; x.objValid = false;
  for (float& r : x.rad) r = NAN;
  x.rangeMm = x.heightMm = NAN;
  return x;
}
void fillFrame(uint16_t v) { for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) FR[s][z] = v; }
// Runs a constant hold from t0 for ms, returning the last output.
SlipOut hold(SlipState& st, const SlipParams& p, uint32_t& t, uint32_t ms, float f) {
  SlipOut o{};
  for (uint32_t e = t + ms; t < e; t += 10) o = slipStep(st, p, in(t, f));
  return o;
}
}  // namespace

TEST(slip_defaults) {
  SlipParams p = slipDefaults();
  CHECK_NEAR(p.fsrDropFrac, 0.30f, 1e-6); CHECK_EQ(p.fsrWindowMs, (uint16_t)300);
  CHECK_EQ(p.tofZoneCount, (uint8_t)4); CHECK_EQ(p.tofConfirmMs, (uint16_t)150);
  CHECK_EQ(p.mode, (uint8_t)2); CHECK_EQ(p.settleMs, (uint16_t)500);
  CHECK_EQ(p.holdoffMs, (uint16_t)1500);
}
TEST(slip_not_armed_before_settle_or_outside_monitor) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0;
  SlipOut o = hold(st, p, t, 400, 2.0f);
  CHECK(!o.armed);
  o = slipStep(st, p, in(t, 0.1f)); t += 10;          // big drop before settle: no event
  CHECK(!o.slip);
  o = slipStep(st, p, in(t, 2.0f, false));             // left MONITOR: reset
  CHECK(!o.armed);
}
TEST(slip_quiet_hold_with_slow_relaxation_no_event) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0; bool any = false;
  for (; t < 30000; t += 10) {                         // -10 % per 10 s
    const float f = 2.0f * (1.0f - 0.01f * (float)t / 1000.0f);
    SlipInput x = in(t, f); x.frameNew = (t % 70) == 0;
    if (slipStep(st, p, x).slip) any = true;
  }
  CHECK(!any);
}
TEST(slip_fast_force_drop_fires_fsr) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0;
  SlipOut o = hold(st, p, t, 1000, 2.0f);
  CHECK(o.armed);
  bool fired = false; uint32_t at = 0;
  for (int k = 0; k < 30; k++, t += 10) {
    const float f = k < 10 ? 2.0f - 0.12f * (float)k : 0.8f;   // -60 % in 100 ms
    o = slipStep(st, p, in(t, f));
    if (o.slip && !fired) { fired = true; at = t; CHECK_EQ((int)o.reason, (int)SLIP_FSR); }
  }
  CHECK(fired);
  CHECK(at <= 1000 + 150);
  CHECK(o.fsrDrop > 0.5f);
}
TEST(slip_holdoff_single_event) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0; hold(st, p, t, 1000, 2.0f);
  int events = 0;
  for (int k = 0; k < 100; k++, t += 10) {              // force keeps oscillating hard for 1 s
    const float f = (k / 5) % 2 ? 0.5f : 2.0f;
    if (slipStep(st, p, in(t, f)).slip) events++;
  }
  CHECK_EQ(events, 1);
}
TEST(slip_frame_change_fires_tof) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0;
  for (; t < 1000; t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (t % 70) == 0; slipStep(st, p, x); }
  for (int z = 0; z < 6; z++) FR[1][z] = 240;           // object moved: 6 zones +40 mm
  SlipOut o{}; bool fired = false;
  for (int k = 0; k < 20; k++, t += 10) {
    SlipInput x = in(t, 2.0f); x.frameNew = (k % 7) == 0;
    o = slipStep(st, p, x);
    if (o.slip) { fired = true; CHECK_EQ((int)o.reason, (int)SLIP_TOF); }
  }
  CHECK(fired);
  CHECK(o.tofZones >= 6);
}
TEST(slip_frame_noise_and_far_zones_ignored) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  for (int z = 0; z < 64; z++) FR[2][z] = 900;          // far background on sensor 3
  uint32_t t = 0;
  for (; t < 1000; t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (t % 70) == 0; slipStep(st, p, x); }
  for (int z = 0; z < 64; z++) { FR[0][z] = 210; FR[2][z] = 500; }   // +10 mm near (< 20 mm limit), far zones move a lot
  bool fired = false;
  for (int k = 0; k < 40; k++, t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (k % 7) == 0; if (slipStep(st, p, x).slip) fired = true; }
  CHECK(!fired);
}
TEST(slip_mode_both_needs_both) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); p.mode = 3; fillFrame(200);
  uint32_t t = 0;
  for (; t < 1000; t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (t % 70) == 0; slipStep(st, p, x); }
  bool fired = false;
  for (int k = 0; k < 30; k++, t += 10) if (slipStep(st, p, in(t, 0.5f)).slip) fired = true;   // FSR only
  CHECK(!fired);
  for (int z = 0; z < 8; z++) FR[0][z] = 260;
  for (int k = 0; k < 30; k++, t += 10) { SlipInput x = in(t, 0.5f); x.frameNew = (k % 7) == 0;
    SlipOut o = slipStep(st, p, x); if (o.slip) { fired = true; CHECK_EQ((int)o.reason, (int)(SLIP_FSR | SLIP_TOF)); } }
  CHECK(fired);
}
TEST(slip_mode_fsr_only_ignores_frames) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); p.mode = 0; fillFrame(200);
  uint32_t t = 0;
  for (; t < 1000; t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (t % 70) == 0; slipStep(st, p, x); }
  for (int z = 0; z < 20; z++) FR[0][z] = 300;
  bool fired = false;
  for (int k = 0; k < 30; k++, t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (k % 7) == 0; if (slipStep(st, p, x).slip) fired = true; }
  CHECK(!fired);
}
TEST(slip_unloaded_pads_ignored) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0; hold(st, p, t, 1000, 0.1f);           // below fsrMinForceN on every pad
  bool fired = false;
  for (int k = 0; k < 30; k++, t += 10) if (slipStep(st, p, in(t, 0.0f)).slip) fired = true;
  CHECK(!fired);
}
TEST(slip_param_names_round_trip) {
  SlipParams p = slipDefaults();
  CHECK(slipParamSet(p, "fsrDropFrac", 0.5f)); CHECK_NEAR(p.fsrDropFrac, 0.5f, 1e-6);
  CHECK(slipParamSet(p, "mode", 3)); CHECK_EQ(p.mode, (uint8_t)3);
  CHECK(!slipParamSet(p, "nope", 1));
  CHECK(std::strcmp(slipParamName(0), "fsrDropFrac") == 0);
  int n = 0; while (slipParamName(n)) n++;
  CHECK_EQ(n, 14);
  CHECK_NEAR(slipParamGet(p, "tofZoneCount"), 4.0f, 1e-6);
}

// Review fixes: a NaN current force counts as 0 (a drop); a single glitch frame does not confirm.
TEST(slip_nan_current_force_is_a_drop) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0; hold(st, p, t, 1000, 2.0f);
  bool fired = false;
  for (int k = 0; k < 10; k++, t += 10) { SlipInput x = in(t, NAN); x.force[1] = 2.0f; if (slipStep(st, p, x).slip) fired = true; }
  CHECK(fired);
}
TEST(slip_tof_single_glitch_record_does_not_confirm) {
  SlipState st; slipReset(st); SlipParams p = slipDefaults(); fillFrame(200);
  uint32_t t = 0;
  for (; t < 1000; t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (t % 70) == 0; slipStep(st, p, x); }
  for (int z = 0; z < 8; z++) FR[0][z] = 0;               // dropout glitch on sensor 1 ...
  bool fired = false;
  for (int k = 0; k < 6; k++, t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = true; if (slipStep(st, p, x).slip) fired = true; }   // 3 records in 30 ms
  for (int z = 0; z < 8; z++) FR[0][z] = 200;             // ... gone again
  for (int k = 0; k < 30; k++, t += 10) { SlipInput x = in(t, 2.0f); x.frameNew = (k % 5) == 0; if (slipStep(st, p, x).slip) fired = true; }
  CHECK(!fired);
}
