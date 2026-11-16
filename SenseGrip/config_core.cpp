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
  c.maxForceN = 3.0f;
  c.detectMm = 80; c.detectHoldMs = 200;
  c.preTouchTimeoutMs = 8000; c.graspTimeoutMs = 5000; c.openTimeoutMs = 15000;
  c.runViaPosition = false;
  for (auto& m : c.tofMask) m = ~0ULL;
  c.fsrEnabled = 0x3F;
  c.fsrFilter = 2; c.fsrFilterWindow = 5; c.fsrFilterAlpha = 0.3f; c.fsrKalmanQ = 0.01f; c.fsrKalmanR = 0.05f;
  c.touchRef = 0; c.touchValid = false;
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
  return ch;
}

const char* limitsProblem(const GripConfig& c) {
  if (!c.limitsValid) return "OPEN zero / close limit not set";
  bool ok = c.closePositive ? (c.closeLimit > c.openZero) : (c.closeLimit < c.openZero);
  if (!ok) return "close limit is not on the closing side of OPEN zero";
  return nullptr;
}

bool limitsRestorable(const GripConfig& c) {
  if (!c.touchValid) return false;
  GripConfig t = c; t.limitsValid = true;
  return limitsProblem(t) == nullptr;
}

FilterParams filterParamsOf(const GripConfig& c) {
  return FilterParams{(FilterKind)c.fsrFilter, c.fsrFilterWindow, c.fsrFilterAlpha, c.fsrKalmanQ, c.fsrKalmanR};
}
