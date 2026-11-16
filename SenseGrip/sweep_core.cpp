#include "sweep_core.h"
#include <cmath>
#include <cstdlib>

namespace {
// Rules (builder's choices, see the phase 2 plan):
constexpr float FINGER_MIN_RANGE_MM = 30.0f;          // reading must change by more than this
constexpr float FINGER_MIN_CORR = 0.8f;               // |Pearson r| with position
constexpr int   MIN_COMPARED = 6;                      // zones needed for estimate / check
constexpr float RANGE_ERR = 0.10f;                     // SEN0628 range error: about 10 % of the reading

bool ok(uint16_t v) { return v > 0 && v < 4000; }
int32_t fromOpen(int32_t pos, int32_t openZero, bool closePositive) { return closePositive ? pos - openZero : openZero - pos; }

void rawToDisplay(uint8_t orientation, uint8_t z, uint8_t& row, uint8_t& col) {
  for (uint8_t r = 0; r < 8; r++) for (uint8_t c = 0; c < 8; c++)
    if (zoneSourceIndex(orientation, r, c) == z) { row = r; col = c; return; }
  row = z / 8; col = z % 8;
}

// RMS of (a-b) over finger zones valid in both; returns count compared. RMS (not mean |diff|)
// so that symmetric noise does not create ties between neighbouring samples.
int compare(const SweepModel& m, const uint16_t a[3][64], uint8_t aValid, const uint16_t b[3][64], uint8_t bValid, float& mean) {
  float sum = 0; int n = 0;
  for (int s = 0; s < 3; s++) {
    if (!((aValid >> s) & 1) || !((bValid >> s) & 1)) continue;
    for (int z = 0; z < 64; z++) {
      if (!((m.fingerZones[s] >> z) & 1)) continue;
      if (!ok(a[s][z]) || !ok(b[s][z])) continue;
      const float d = (float)a[s][z] - (float)b[s][z];
      sum += d * d; n++;
    }
  }
  mean = n ? std::sqrt(sum / n) : 0;
  return n;
}
}  // namespace

bool sweepValid(const SweepSample* s, int n, int32_t openZero, bool closePositive) {
  if (n < 20 || n > SWEEP_MAX) return false;
  const int32_t d0 = fromOpen(s[0].pos, openZero, closePositive);
  if (d0 > 150 || d0 < -150) return false;
  for (int i = 1; i < n; i++)
    if (fromOpen(s[i].pos, openZero, closePositive) <= fromOpen(s[i - 1].pos, openZero, closePositive)) return false;
  return true;
}

void sweepAnalyze(const SweepSample* s, int n, const SensorCalib cal[3], int32_t openZero,
                  int32_t touchRef, bool closePositive, SweepModel& out) {
  out.n = n; out.openZero = openZero; out.touchRef = touchRef;
  // 1. Finger zones: large, position-correlated change beyond the invisible region.
  for (int sn = 0; sn < 3; sn++) {
    out.fingerZones[sn] = 0;
    for (int z = 0; z < 64; z++) {
      double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0; int k = 0; float lo = 1e9f, hi = -1e9f;
      for (int i = 0; i < n; i++) {
        if (fromOpen(s[i].pos, openZero, closePositive) <= FINGERS_INVISIBLE_COUNTS) continue;
        if (!((s[i].valid >> sn) & 1) || !ok(s[i].mm[sn][z])) continue;
        const double x = s[i].pos, y = s[i].mm[sn][z];
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; k++;
        if (y < lo) lo = (float)y;
        if (y > hi) hi = (float)y;
      }
      if (k < 10 || hi - lo <= FINGER_MIN_RANGE_MM) continue;
      const double cov = sxy - sx * sy / k, vx = sxx - sx * sx / k, vy = syy - sy * sy / k;
      if (vx <= 0 || vy <= 0) continue;
      if (std::fabs(cov / std::sqrt(vx * vy)) >= FINGER_MIN_CORR) out.fingerZones[sn] |= 1ULL << z;
    }
  }
  // 2. Opening per sample: innermost finger point (fingertips are the innermost finger parts).
  //    A finger zone counts only once the finger is in it: outside the invisible region and
  //    clearly away from what it read at OPEN (s[0], fingers not in view) - more than 30 mm and
  //    more than twice the 10 % range error. Otherwise it is background, not a fingertip.
  for (int i = 0; i < n; i++) {
    float best = NAN;
    if (fromOpen(s[i].pos, openZero, closePositive) <= FINGERS_INVISIBLE_COUNTS) { out.opening[i] = best; continue; }
    for (int sn = 0; sn < 3; sn++) {
      if (!((s[i].valid >> sn) & 1)) continue;
      for (int z = 0; z < 64; z++) {
        if (!((out.fingerZones[sn] >> z) & 1) || !ok(s[i].mm[sn][z])) continue;
        if (((s[0].valid >> sn) & 1) && ok(s[0].mm[sn][z])) {
          const float bg = s[0].mm[sn][z], d = std::fabs((float)s[i].mm[sn][z] - bg);
          if (d <= FINGER_MIN_RANGE_MM || d <= 2 * RANGE_ERR * bg) continue;
        }
        uint8_t row, col; rawToDisplay(cal[sn].orientation, (uint8_t)z, row, col);
        Vec3 p;
        if (!zonePoint((uint8_t)sn, row, col, s[i].mm[sn][z], cal[sn], p)) continue;
        const float r = horizontalRadius(p);
        if (!(r >= best)) best = r;                    // also replaces NaN
      }
    }
    out.opening[i] = best;
  }
  // 3. Fill gaps: interpolate inside, extrapolate the open end to OPENING_AT_OPEN_MM at openZero,
  //    hold the last value at the closed end.
  int first = -1, last = -1;
  for (int i = 0; i < n; i++) if (!std::isnan(out.opening[i])) { if (first < 0) first = i; last = i; }
  out.firstSeen = first;
  if (first < 0) { for (int i = 0; i < n; i++) out.opening[i] = OPENING_AT_OPEN_MM; return; }
  const float d0 = (float)fromOpen(s[first].pos, openZero, closePositive);
  for (int i = 0; i < first; i++) {
    const float d = (float)fromOpen(s[i].pos, openZero, closePositive);
    const float t = d0 > 0 ? d / d0 : 1;
    out.opening[i] = OPENING_AT_OPEN_MM + (out.opening[first] - OPENING_AT_OPEN_MM) * t;
  }
  for (int i = first + 1; i <= last; i++) {
    if (!std::isnan(out.opening[i])) continue;
    int j = i; while (std::isnan(out.opening[j])) j++;
    const float a = out.opening[i - 1], b = out.opening[j];
    for (int k = i; k < j; k++) out.opening[k] = a + (b - a) * (float)(k - i + 1) / (float)(j - i + 1);
    i = j;
  }
  for (int i = last + 1; i < n; i++) out.opening[i] = out.opening[last];
}

float openingAt(const SweepModel& m, const SweepSample* s, int32_t pos) {
  if (m.n <= 0) return NAN;
  const bool inc = m.n < 2 || s[m.n - 1].pos > s[0].pos;
  auto before = [&](int32_t a, int32_t b) { return inc ? a <= b : a >= b; };
  if (before(pos, s[0].pos)) {
    // Between openZero and the first sample: interpolate from OPENING_AT_OPEN_MM.
    const float span = (float)(s[0].pos - m.openZero);
    if (span == 0) return m.opening[0];
    float t = (float)(pos - m.openZero) / span;
    if (t < 0) t = 0;
    return OPENING_AT_OPEN_MM + (m.opening[0] - OPENING_AT_OPEN_MM) * t;
  }
  for (int i = 1; i < m.n; i++) {
    if (before(pos, s[i].pos)) {
      const float t = (float)(pos - s[i - 1].pos) / (float)(s[i].pos - s[i - 1].pos);
      return m.opening[i - 1] + (m.opening[i] - m.opening[i - 1]) * t;
    }
  }
  return m.opening[m.n - 1];
}

PosEstimate estimatePosition(const SweepModel& m, const SweepSample* s, const uint16_t frame[3][64], uint8_t frameValid) {
  PosEstimate e{false, 0, 0};
  float bestScore = 1e9f; int best = -1;
  for (int i = 0; i < m.n; i++) {
    float mean;
    if (compare(m, frame, frameValid, s[i].mm, s[i].valid, mean) < MIN_COMPARED) continue;
    if (mean < bestScore) { bestScore = mean; best = i; }
  }
  if (best < 0) return e;
  const bool closePositive = m.n < 2 || s[m.n - 1].pos > s[0].pos;
  if (fromOpen(s[best].pos, m.openZero, closePositive) <= FINGERS_INVISIBLE_COUNTS) return e;
  e.valid = true; e.pos = s[best].pos; e.score = bestScore;
  return e;
}

namespace {
int nearestSample(const SweepModel& m, const SweepSample* s, int32_t pos) {
  int near = 0;
  for (int i = 1; i < m.n; i++)
    if (std::labs((long)s[i].pos - pos) < std::labs((long)s[near].pos - pos)) near = i;
  return near;
}
}  // namespace

bool viewObstructed(const SweepModel& m, const SweepSample* s, int32_t pos, const uint16_t frame[3][64], uint8_t frameValid) {
  if (m.n <= 0) return false;
  const SweepSample& ref = s[nearestSample(m, s, pos)];
  int closer = 0;
  for (int sn = 0; sn < 3; sn++) {
    if (!((frameValid >> sn) & 1) || !((ref.valid >> sn) & 1)) continue;
    for (int z = 0; z < 64; z++) {
      if ((m.fingerZones[sn] >> z) & 1) continue;
      const uint16_t f = frame[sn][z], r = ref.mm[sn][z];
      if (!ok(f)) continue;
      // 50 mm, or twice the 10 % range error for far background (two readings, each +-10 %).
      if (!ok(r) || (float)f < (float)r - std::fmax(50.0f, 2 * RANGE_ERR * r)) closer++;
    }
  }
  return closer >= 3;
}

bool positionMismatch(const SweepModel& m, const SweepSample* s, int32_t pos, const uint16_t frame[3][64],
                      uint8_t frameValid, float thresholdMm) {
  if (m.n <= 0) return false;
  const SweepSample& ref = s[nearestSample(m, s, pos)];
  // RMS of the difference in units of the per-zone tolerance: thresholdMm, or the 10 % range
  // error of the recorded reading when that is larger (far zones).
  double sum = 0; int n = 0;
  for (int sn = 0; sn < 3; sn++) {
    if (!((frameValid >> sn) & 1) || !((ref.valid >> sn) & 1)) continue;
    for (int z = 0; z < 64; z++) {
      if (!((m.fingerZones[sn] >> z) & 1) || !ok(frame[sn][z]) || !ok(ref.mm[sn][z])) continue;
      const float tol = std::fmax(thresholdMm, RANGE_ERR * ref.mm[sn][z]);
      const float e = ((float)frame[sn][z] - (float)ref.mm[sn][z]) / tol;
      sum += e * e; n++;
    }
  }
  if (n < MIN_COMPARED) return false;
  return std::sqrt(sum / n) > 1.0;
}
