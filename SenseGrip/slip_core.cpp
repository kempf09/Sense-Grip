#include "slip_core.h"
#include <cmath>
#include <cstring>

SlipParams slipDefaults() {
  SlipParams p;
  p.fsrDropFrac = 0.30f; p.fsrWindowMs = 300; p.fsrMinForceN = 0.15f; p.fsrConfirmMs = 30;
  p.tofNearMm = 350; p.tofZoneMm = 12; p.tofZoneFrac = 0.10f; p.tofZoneCount = 4; p.tofConfirmMs = 150;
  p.tofDispMm = 0.0f; p.mode = 2; p.fuseWindowMs = 500; p.settleMs = 500; p.holdoffMs = 1500;
  return p;
}

namespace {
const char* const NAMES[] = {"fsrDropFrac", "fsrWindowMs", "fsrMinForceN", "fsrConfirmMs", "tofNearMm", "tofZoneMm",
                             "tofZoneFrac", "tofZoneCount", "tofConfirmMs", "tofDispMm", "mode", "fuseWindowMs",
                             "settleMs", "holdoffMs"};
constexpr int NN = sizeof(NAMES) / sizeof(NAMES[0]);
int idx(const char* n) { for (int i = 0; i < NN; i++) if (std::strcmp(n, NAMES[i]) == 0) return i; return -1; }
uint16_t u16(float v) { return (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v + 0.5f); }
uint8_t u8(float v) { return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v + 0.5f); }
bool okMm(uint16_t v) { return v > 0 && v < 4000; }
}  // namespace

const char* slipParamName(int i) { return (i >= 0 && i < NN) ? NAMES[i] : nullptr; }

bool slipParamSet(SlipParams& p, const char* name, float v) {
  switch (idx(name)) {
    case 0: p.fsrDropFrac = v; return true;
    case 1: p.fsrWindowMs = u16(v); return true;
    case 2: p.fsrMinForceN = v; return true;
    case 3: p.fsrConfirmMs = u16(v); return true;
    case 4: p.tofNearMm = u16(v); return true;
    case 5: p.tofZoneMm = u16(v); return true;
    case 6: p.tofZoneFrac = v; return true;
    case 7: p.tofZoneCount = u8(v); return true;
    case 8: p.tofConfirmMs = u16(v); return true;
    case 9: p.tofDispMm = v; return true;
    case 10: p.mode = u8(v); return true;
    case 11: p.fuseWindowMs = u16(v); return true;
    case 12: p.settleMs = u16(v); return true;
    case 13: p.holdoffMs = u16(v); return true;
  }
  return false;
}

float slipParamGet(const SlipParams& p, const char* name) {
  switch (idx(name)) {
    case 0: return p.fsrDropFrac; case 1: return p.fsrWindowMs; case 2: return p.fsrMinForceN;
    case 3: return p.fsrConfirmMs; case 4: return p.tofNearMm; case 5: return p.tofZoneMm;
    case 6: return p.tofZoneFrac; case 7: return p.tofZoneCount; case 8: return p.tofConfirmMs;
    case 9: return p.tofDispMm; case 10: return p.mode; case 11: return p.fuseWindowMs;
    case 12: return p.settleMs; case 13: return p.holdoffMs;
  }
  return NAN;
}

void slipReset(SlipState& s) { std::memset(&s, 0, sizeof s); }

SlipOut slipStep(SlipState& s, const SlipParams& p, const SlipInput& in) {
  SlipOut o{}; o.tofDispMm = NAN;
  if (!in.active) { slipReset(s); return o; }
  const uint32_t t = in.tMs;
  if (!s.wasActive) { slipReset(s); s.wasActive = true; s.activeSince = t; }
  // Force history (time-stamped ring) and the latest frame set.
  s.histT[s.histHead] = t;
  for (int i = 0; i < 6; i++) s.hist[s.histHead][i] = in.force[i];
  s.histHead = (s.histHead + 1) % SLIP_HIST;
  if (s.histN < SLIP_HIST) s.histN++;
  if (in.frame && (in.frameNew || !s.haveFrame)) s.haveFrame = true;

  if (!s.armed) {
    if ((uint32_t)(t - s.activeSince) < p.settleMs) return o;
    s.armed = true;
    for (int i = 0; i < 6; i++) s.base[i] = in.force[i];
    if (s.haveFrame && in.frame) { std::memcpy(s.frame0, in.frame, sizeof s.frame0); s.valid0 = in.frameValid; s.baseFrame = true; }
    s.obj0 = in.objValid;
    if (in.objValid) { s.range0 = in.rangeMm; s.height0 = in.heightMm; for (int k = 0; k < 3; k++) s.rad0[k] = in.rad[k]; }
  }
  o.armed = true;
  // After an event the object may rest in a new place / at a lower force: once the holdoff is
  // over, that becomes the new baseline (otherwise it would re-fire every holdoff).
  if (s.rebasePending && (int32_t)(t - s.rebaseAt) >= 0) {
    s.rebasePending = false;
    for (int i = 0; i < 6; i++) s.base[i] = in.force[i];
    if (s.haveFrame && in.frame) { std::memcpy(s.frame0, in.frame, sizeof s.frame0); s.valid0 = in.frameValid; s.baseFrame = true; }
    s.obj0 = in.objValid;
    if (in.objValid) { s.range0 = in.rangeMm; s.height0 = in.heightMm; for (int k = 0; k < 3; k++) s.rad0[k] = in.rad[k]; }
    s.tofOver = false; s.fsrOver = false; s.tofZones = 0;
    s.haveFsrFlag = s.haveTofFlag = false;
  }

  // FSR: largest relative drop from the window maximum, for pads loaded at baseline.
  const uint32_t win = p.fsrWindowMs > 600 ? 600 : p.fsrWindowMs;
  float drop = 0;
  for (int i = 0; i < 6; i++) {
    const float b = s.base[i];
    if (!(b >= p.fsrMinForceN)) continue;
    const float cur = in.force[i] == in.force[i] ? in.force[i] : 0.0f;   // NaN (pad gone) = 0
    float mx = cur;
    for (int k = 0; k < s.histN; k++) {
      const int j = (s.histHead - 1 - k + 2 * SLIP_HIST) % SLIP_HIST;
      if ((uint32_t)(t - s.histT[j]) > win) break;
      const float v = s.hist[j][i];
      if (v > mx) mx = v;                          // NaN never wins
    }
    const float d = (mx - cur) / b;
    if (d > drop) drop = d;
  }
  o.fsrDrop = drop;
  if (drop >= p.fsrDropFrac) {
    if (!s.fsrOver) { s.fsrOver = true; s.fsrSince = t; }
    if ((uint32_t)(t - s.fsrSince) >= p.fsrConfirmMs) o.fsrFlag = true;
  } else s.fsrOver = false;

  // ToF: zones changed against the baseline frame (the fingers do not move in MONITOR).
  if (!s.baseFrame && in.frame && in.frameNew) {        // first frame after settle becomes the baseline
    std::memcpy(s.frame0, in.frame, sizeof s.frame0); s.valid0 = in.frameValid; s.baseFrame = true;
  } else if (s.baseFrame && in.frame && in.frameNew) {
    int n = 0;
    for (int sn = 0; sn < 3; sn++) {
      if (!((s.valid0 >> sn) & 1) || !((in.frameValid >> sn) & 1)) continue;
      for (int z = 0; z < 64; z++) {
        const uint16_t b = s.frame0[sn][z];
        if (!okMm(b) || b > p.tofNearMm) continue;
        const uint16_t v = in.frame[sn][z];
        const float lim = std::fmax((float)p.tofZoneMm, p.tofZoneFrac * (float)b);
        if (!okMm(v) || std::fabs((float)v - (float)b) > lim) n++;
      }
    }
    s.tofZones = n;
    if (n >= p.tofZoneCount) { if (!s.tofOver) { s.tofOver = true; s.tofSince = t; } }
    else s.tofOver = false;
  }
  o.tofZones = s.tofZones;
  // Time-based confirmation: frame records come per sensor (3 per frame set), so a record count
  // would not mean "N frames"; a single glitch frame lasts less than tofConfirmMs.
  bool tof = s.tofOver && (uint32_t)(t - s.tofSince) >= p.tofConfirmMs;
  if (s.obj0 && in.objValid) {
    float d = std::fabs(in.rangeMm - s.range0);
    d = std::fmax(d, std::fabs(in.heightMm - s.height0));
    for (int k = 0; k < 3; k++) if (in.rad[k] == in.rad[k] && s.rad0[k] == s.rad0[k]) d = std::fmax(d, std::fabs(in.rad[k] - s.rad0[k]));
    o.tofDispMm = d;
    if (p.tofDispMm > 0 && d >= std::fmax(p.tofDispMm, 0.1f * s.range0)) tof = true;
  }
  o.tofFlag = tof;

  // Fusion and event.
  if (o.fsrFlag) { s.haveFsrFlag = true; s.lastFsrMs = t; }
  if (o.tofFlag) { s.haveTofFlag = true; s.lastTofMs = t; }
  bool cand = false;
  switch (p.mode) {
    case 0: cand = o.fsrFlag; break;
    case 1: cand = o.tofFlag; break;
    case 3: cand = (o.fsrFlag || o.tofFlag) && s.haveFsrFlag && s.haveTofFlag &&
                   (uint32_t)(t - s.lastFsrMs) <= p.fuseWindowMs && (uint32_t)(t - s.lastTofMs) <= p.fuseWindowMs;
            break;
    default: cand = o.fsrFlag || o.tofFlag; break;
  }
  o.reason = (uint8_t)((o.fsrFlag ? SLIP_FSR : 0) | (o.tofFlag ? SLIP_TOF : 0));
  if (p.mode == 3 && cand) o.reason = SLIP_FSR | SLIP_TOF;
  if (cand && (!s.holdActive || (int32_t)(t - s.holdUntil) >= 0)) {
    o.slip = true; s.holdActive = true; s.holdUntil = t + p.holdoffMs;
    s.rebasePending = true; s.rebaseAt = t + p.holdoffMs;
  }
  return o;
}
