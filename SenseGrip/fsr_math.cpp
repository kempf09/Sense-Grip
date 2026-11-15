#include "fsr_math.h"
#include "pins.h"
#include <cmath>

static bool okF(float f, float lo, float hi) { return std::isfinite(f) && f >= lo && f <= hi; }

bool fsrCalValid(const FsrCal& c) {
  if (!std::isfinite(c.fixedOhms) || c.fixedOhms <= 0) return false;
  if (!okF(c.f1, 0.2f, 30.0f) || !okF(c.f2, 0.2f, 30.0f)) return false;
  if (!std::isfinite(c.o1) || !std::isfinite(c.o2)) return false;
  if (!(c.o1 > 0 && c.o1 < 10e6f && c.o2 > 0 && c.o2 < 10e6f)) return false;
  if (c.f1 == c.f2 || c.o1 == c.o2) return false;
  if (!okF(c.zeroMv, 0.0f, 1000.0f)) return false;
  return true;
}

float fsrOhms(float mv, float fixedOhms) {
  if (!(mv >= 1.0f)) return 10e6f;
  if (mv >= 3299.0f) return 0.0f;
  return fixedOhms * (3300.0f / mv - 1.0f);
}

float fsrForce(const FsrCal& c, float ohms) {
  if (!fsrCalValid(c)) return NAN;
  if (!(ohms > 0) || ohms >= 10e6f) return 0.0f;
  float b = std::log(c.f2 / c.f1) / std::log(c.o2 / c.o1);
  float a = c.f1 / std::pow(c.o1, b);
  return a * std::pow(ohms, b);
}

void updateContacts(ContactDebounce& d, const float force[6], float thresholdN, uint8_t samples) {
  for (int f = 0; f < 3; f++) {
    bool touching = false;
    for (int k = 0; k < 2; k++) {
      float v = force[FINGER_FSR_MAP[f][k]];
      if (std::isfinite(v) && v >= thresholdN) touching = true;
    }
    if (touching) {
      if (d.count[f] < 255) d.count[f]++;
    } else {
      d.count[f] = 0;
    }
    d.stable[f] = touching && d.count[f] >= samples;
  }
}

FsrAggregate aggregate(const float force[6], float thresholdN) {
  FsrAggregate a{};
  float sum = 0;
  for (int i = 0; i < FSR_COUNT; i++) {
    float v = force[i];
    if (!std::isfinite(v)) continue;
    if (v > a.maxForce) a.maxForce = v;
    if (v >= thresholdN) { a.channelsInContact++; sum += v; }
  }
  a.avgContactForce = a.channelsInContact ? sum / a.channelsInContact : 0.0f;
  for (int f = 0; f < 3; f++) {
    float m = 0;
    for (int k = 0; k < 2; k++) {
      float v = force[FINGER_FSR_MAP[f][k]];
      if (std::isfinite(v) && v > m) m = v;
    }
    a.fingerForce[f] = m;
  }
  return a;
}

void maskForces(const float force[6], uint8_t enabledMask, float out[6]) {
  for (int i = 0; i < 6; i++) out[i] = (enabledMask & (1u << i)) ? force[i] : 0.0f;
}

float limitForceOf(const float raw[6], const float filtered[6], uint8_t enabledMask) {
  float m = 0;
  for (int i = 0; i < 6; i++) {
    if (!(enabledMask & (1u << i))) continue;
    if (raw[i] == raw[i] && raw[i] > m) m = raw[i];
    if (filtered[i] == filtered[i] && filtered[i] > m) m = filtered[i];
  }
  return m;
}
