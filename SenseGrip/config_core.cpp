#include "config_core.h"
#include <cmath>

GripConfig defaultConfig() {
  GripConfig c{};
  c.openZero = 0; c.closeLimit = 0; c.limitsValid = false; c.closePositive = true;
  c.approachSpeed = 61; c.graspSpeed = 10; c.openSpeed = 61;
  c.closeTorqueLimit = 200; c.openTorqueLimit = 260;
  c.openToleranceCounts = 8; c.limitSlackCounts = 200;
  c.contactForceN = 0.25f; c.contactSamples = 3;
  c.graspForceN = 1.0f; c.graspSamples = 3;
  c.minActiveFsr = 2;
  c.maxForceN = 8.0f;   // above the large class force (4 N); was 3 N before v7
  c.detectMm = 80; c.detectHoldMs = 200;
  c.preTouchTimeoutMs = 8000; c.graspTimeoutMs = 5000; c.openTimeoutMs = 15000;
  c.runViaPosition = false;
  for (auto& m : c.tofMask) m = ~0ULL;
  c.fsrEnabled = 0x3F;
  c.fsrFilter = 2; c.fsrFilterWindow = 5; c.fsrFilterAlpha = 0.3f; c.fsrKalmanQ = 0.01f; c.fsrKalmanR = 0.05f;
  c.touchRef = 0; c.touchValid = false;
  c.smallMaxMm = 50; c.mediumMaxMm = 90; c.contactSpeed = 15;
  c.smallStandoffMm = 10;    c.smallForceN = 1.0f;    c.smallMaxCurrent = 200;    c.smallGraspSpeed = 5;
  c.mediumStandoffMm = 10;   c.mediumForceN = 2.0f;   c.mediumMaxCurrent = 260;   c.mediumGraspSpeed = 5;
  c.largeStandoffMm = 10;    c.largeForceN = 4.0f;    c.largeMaxCurrent = 350;    c.largeGraspSpeed = 5;
  c.delicateStandoffMm = 15; c.delicateForceN = 0.4f; c.delicateMaxCurrent = 150; c.delicateGraspSpeed = 3;
  c.sweepStepCounts = 100; c.sweepSpeed = 5; c.sweepCurrent = 0;
  const SlipParams sp = slipDefaults();
  c.slipFsrDropFrac = sp.fsrDropFrac; c.slipFsrWindowMs = sp.fsrWindowMs; c.slipFsrMinForceN = sp.fsrMinForceN;
  c.slipFsrConfirmMs = sp.fsrConfirmMs; c.slipTofNearMm = sp.tofNearMm; c.slipTofZoneMm = sp.tofZoneMm;
  c.slipTofZoneFrac = sp.tofZoneFrac; c.slipTofZoneCount = sp.tofZoneCount; c.slipTofConfirmMs = sp.tofConfirmMs;
  c.slipTofDispMm = sp.tofDispMm; c.slipMode = sp.mode; c.slipFuseWindowMs = sp.fuseWindowMs;
  c.slipSettleMs = sp.settleMs; c.slipHoldoffMs = sp.holdoffMs; c.slipTighten = false;
  return c;
}

namespace {
template <typename T>
bool clampInt(T& v, long long lo, long long hi) {
  long long x = (long long)v;
  if (x < lo) { v = (T)lo; return true; }
  if (x > hi) { v = (T)hi; return true; }
  return false;
}
bool clampF(float& v, float lo, float hi, float fallback) {
  if (!std::isfinite(v)) { v = fallback; return true; }
  if (v < lo) { v = lo; return true; }
  if (v > hi) { v = hi; return true; }
  return false;
}
}  // namespace

bool clampConfig(GripConfig& c) {
  const GripConfig d = defaultConfig();
  bool ch = false;
  ch |= clampInt(c.openZero, -32767, 32767);
  ch |= clampInt(c.closeLimit, -32767, 32767);
  if (c.fsrEnabled & ~0x3F) { c.fsrEnabled &= 0x3F; ch = true; }
  if (c.fsrFilter > 4) { c.fsrFilter = d.fsrFilter; ch = true; }
  ch |= clampInt(c.fsrFilterWindow, 2, FILTER_MAX_WINDOW);
  ch |= clampF(c.fsrFilterAlpha, 0.05f, 1.0f, d.fsrFilterAlpha);
  ch |= clampF(c.fsrKalmanQ, 1e-5f, 10.0f, d.fsrKalmanQ);
  ch |= clampF(c.fsrKalmanR, 1e-5f, 10.0f, d.fsrKalmanR);
  ch |= clampInt(c.approachSpeed, 1, SPEED_MAX);
  ch |= clampInt(c.graspSpeed, 1, SPEED_MAX);
  ch |= clampInt(c.openSpeed, 1, SPEED_MAX);
  ch |= clampInt(c.closeTorqueLimit, 1, 430);
  ch |= clampInt(c.openTorqueLimit, 1, 430);
  ch |= clampInt(c.openToleranceCounts, 1, 200);
  ch |= clampInt(c.limitSlackCounts, 0, 2000);
  ch |= clampF(c.contactForceN, 0.05f, 5.0f, d.contactForceN);
  ch |= clampInt(c.contactSamples, 1, 20);
  ch |= clampF(c.graspForceN, 0.1f, 30.0f, d.graspForceN);
  ch |= clampInt(c.graspSamples, 1, 20);
  ch |= clampInt(c.minActiveFsr, 1, 6);
  ch |= clampF(c.maxForceN, 0.5f, 30.0f, d.maxForceN);
  ch |= clampInt(c.detectMm, 20, 1000);
  ch |= clampInt(c.detectHoldMs, 0, 2000);
  ch |= clampInt(c.preTouchTimeoutMs, 500, 60000);
  ch |= clampInt(c.graspTimeoutMs, 500, 60000);
  ch |= clampInt(c.openTimeoutMs, 500, 60000);
  ch |= clampInt(c.smallMaxMm, 20, 250);
  ch |= clampInt(c.mediumMaxMm, 20, 250);
  ch |= clampInt(c.contactSpeed, 1, SPEED_MAX);
  struct P { float* st; float* f; uint16_t* cur; uint16_t* sp; float dst, df; };
  const P ps[4] = {{&c.smallStandoffMm, &c.smallForceN, &c.smallMaxCurrent, &c.smallGraspSpeed, d.smallStandoffMm, d.smallForceN},
                   {&c.mediumStandoffMm, &c.mediumForceN, &c.mediumMaxCurrent, &c.mediumGraspSpeed, d.mediumStandoffMm, d.mediumForceN},
                   {&c.largeStandoffMm, &c.largeForceN, &c.largeMaxCurrent, &c.largeGraspSpeed, d.largeStandoffMm, d.largeForceN},
                   {&c.delicateStandoffMm, &c.delicateForceN, &c.delicateMaxCurrent, &c.delicateGraspSpeed, d.delicateStandoffMm, d.delicateForceN}};
  ch |= clampF(c.slipFsrDropFrac, 0.05f, 0.95f, d.slipFsrDropFrac);
  ch |= clampInt(c.slipFsrWindowMs, 50, 600);
  ch |= clampF(c.slipFsrMinForceN, 0.0f, 10.0f, d.slipFsrMinForceN);
  ch |= clampInt(c.slipFsrConfirmMs, 0, 500);
  ch |= clampInt(c.slipTofNearMm, 50, 2000);
  ch |= clampInt(c.slipTofZoneMm, 1, 500);
  ch |= clampF(c.slipTofZoneFrac, 0.0f, 1.0f, d.slipTofZoneFrac);
  ch |= clampInt(c.slipTofZoneCount, 1, 192);
  ch |= clampInt(c.slipTofConfirmMs, 0, 2000);
  ch |= clampF(c.slipTofDispMm, 0.0f, 200.0f, d.slipTofDispMm);
  if (c.slipMode > 3) { c.slipMode = d.slipMode; ch = true; }
  ch |= clampInt(c.slipFuseWindowMs, 0, 5000);
  ch |= clampInt(c.slipSettleMs, 0, 5000);
  ch |= clampInt(c.slipHoldoffMs, 0, 10000);
  ch |= clampInt(c.sweepStepCounts, 60, 500);
  ch |= clampInt(c.sweepSpeed, 1, 20);
  if (c.sweepCurrent != 0) ch |= clampInt(c.sweepCurrent, 50, 415);
  for (const P& p : ps) {
    ch |= clampF(*p.st, 0.0f, 50.0f, p.dst);
    ch |= clampF(*p.f, 0.1f, 30.0f, p.df);
    ch |= clampInt(*p.cur, 50, 415);
    ch |= clampInt(*p.sp, 1, SPEED_MAX);
  }
  return ch;
}

const char* limitsProblem(const GripConfig& c) {
  if (!c.limitsValid) return "OPEN zero / close limit not set";
  bool ok = c.closePositive ? (c.closeLimit > c.openZero) : (c.closeLimit < c.openZero);
  if (!ok) return "close limit is not on the closing side of OPEN zero";
  return nullptr;
}

ClassProfile profileOf(const GripConfig& c, ObjClass k) {
  switch (k) {
    case ObjClass::SMALL:    return {c.smallStandoffMm, c.smallForceN, c.smallMaxCurrent, c.smallGraspSpeed};
    case ObjClass::MEDIUM:   return {c.mediumStandoffMm, c.mediumForceN, c.mediumMaxCurrent, c.mediumGraspSpeed};
    case ObjClass::LARGE:    return {c.largeStandoffMm, c.largeForceN, c.largeMaxCurrent, c.largeGraspSpeed};
    case ObjClass::DELICATE: break;
  }
  return {c.delicateStandoffMm, c.delicateForceN, c.delicateMaxCurrent, c.delicateGraspSpeed};
}

const char* profileProblem(const GripConfig& c) {
  if (c.smallMaxMm >= c.mediumMaxMm) return "small/medium width limit must be below the medium/large limit";
  static const char* const BELOW_MAX[4] = {
      "small target force must be below Max force N (FSR tab)", "medium target force must be below Max force N (FSR tab)",
      "large target force must be below Max force N (FSR tab)", "delicate target force must be below Max force N (FSR tab)"};
  static const char* const ABOVE_CONTACT[4] = {
      "small target force must be above Contact force N", "medium target force must be above Contact force N",
      "large target force must be above Contact force N", "delicate target force must be above Contact force N"};
  for (int k = 0; k < 4; k++) {
    const ClassProfile p = profileOf(c, (ObjClass)k);
    if (!(p.forceN > c.contactForceN)) return ABOVE_CONTACT[k];
    if (!(p.forceN < c.maxForceN)) return BELOW_MAX[k];
  }
  return nullptr;
}

bool upgradeConfig(GripConfig& c, uint8_t fromVersion) {
  bool ch = false;
  const GripConfig d = defaultConfig();
  if (fromVersion < 9 && c.maxForceN < d.maxForceN) { c.maxForceN = d.maxForceN; ch = true; }
  return ch;
}

SlipParams slipParamsOf(const GripConfig& c) {
  SlipParams p;
  p.fsrDropFrac = c.slipFsrDropFrac; p.fsrWindowMs = c.slipFsrWindowMs; p.fsrMinForceN = c.slipFsrMinForceN;
  p.fsrConfirmMs = c.slipFsrConfirmMs; p.tofNearMm = c.slipTofNearMm; p.tofZoneMm = c.slipTofZoneMm;
  p.tofZoneFrac = c.slipTofZoneFrac; p.tofZoneCount = c.slipTofZoneCount; p.tofConfirmMs = c.slipTofConfirmMs;
  p.tofDispMm = c.slipTofDispMm; p.mode = c.slipMode; p.fuseWindowMs = c.slipFuseWindowMs;
  p.settleMs = c.slipSettleMs; p.holdoffMs = c.slipHoldoffMs;
  return p;
}

bool limitsRestorable(const GripConfig& c) {
  if (!c.touchValid) return false;
  GripConfig t = c; t.limitsValid = true;
  return limitsProblem(t) == nullptr;
}

FilterParams filterParamsOf(const GripConfig& c) {
  return FilterParams{(FilterKind)c.fsrFilter, c.fsrFilterWindow, c.fsrFilterAlpha, c.fsrKalmanQ, c.fsrKalmanR};
}
