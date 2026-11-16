#include "object_core.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr float AXIS_MAX_MM = 100.0f, PALM_Z_MM = 5.0f;
constexpr float BG_MARGIN_MM = 15.0f, BG_MARGIN_FRAC = 0.20f;   // 2 x the 10 % range error

void rawToDisplay(uint8_t orientation, uint8_t z, uint8_t& row, uint8_t& col) {
  for (uint8_t r = 0; r < 8; r++) for (uint8_t c = 0; c < 8; c++)
    if (zoneSourceIndex(orientation, r, c) == z) { row = r; col = c; return; }
  row = z / 8; col = z % 8;
}
float median(float* v, int n) {
  std::sort(v, v + n);
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}
}  // namespace

ObjectMeasure measureObject(const uint16_t frame[3][64], uint8_t frameValid,
                            const uint16_t background[3][64], const uint64_t fingerZones[3],
                            const SensorCalib cal[3]) {
  ObjectMeasure m{};
  m.shape = Shape::UNKNOWN;
  float ranges[192]; int nr = 0;
  float zmin = 1e9f, zmax = -1e9f, sumR = 0; int sensorsWithR = 0;
  for (int s = 0; s < 3; s++) {
    m.sensorRadiusMm[s] = NAN;
    if (!((frameValid >> s) & 1)) continue;
    float radii[64]; int k = 0;
    for (int z = 0; z < 64; z++) {
      if ((fingerZones[s] >> z) & 1) continue;
      const uint16_t v = frame[s][z], bg = background[s][z];
      if (v == 0 || v >= 4000) continue;
      if (bg > 0 && bg < 4000 && !((float)v < (float)bg - std::fmax(BG_MARGIN_MM, BG_MARGIN_FRAC * bg))) continue;
      uint8_t row, col; rawToDisplay(cal[s].orientation, (uint8_t)z, row, col);
      Vec3 p;
      if (!zonePoint((uint8_t)s, row, col, v, cal[s], p)) continue;
      const float r = horizontalRadius(p);
      if (r > AXIS_MAX_MM || p.z <= PALM_Z_MM) continue;
      radii[k++] = r;
      ranges[nr++] = v;
      m.zones[s] |= 1ULL << z;
      zmin = std::fmin(zmin, p.z); zmax = std::fmax(zmax, p.z);
    }
    if (k >= 2) { m.sensorRadiusMm[s] = median(radii, k); sumR += m.sensorRadiusMm[s]; sensorsWithR++; }
  }
  m.nPoints = (uint8_t)nr;
  m.valid = nr >= 3 && sensorsWithR > 0;
  if (!m.valid) return m;
  m.widthMm = 2 * sumR / sensorsWithR;
  m.rangeMm = median(ranges, nr);
  m.heightMm = zmax - zmin;
  if (sensorsWithR < 2) m.shape = Shape::UNKNOWN;
  else if (m.heightMm >= 1.8f * m.widthMm) m.shape = Shape::ELONGATED;
  else {
    float lo = 1e9f, hi = 0;
    for (float r : m.sensorRadiusMm) if (!std::isnan(r)) { lo = std::fmin(lo, r); hi = std::fmax(hi, r); }
    m.shape = (lo > 0 && hi / lo <= 1.25f) ? Shape::ROUND : Shape::IRREGULAR;
  }
  return m;
}

ObjClass classify(float widthMm, uint16_t smallMaxMm, uint16_t mediumMaxMm) {
  if (widthMm >= 0.9f * mediumMaxMm) return ObjClass::LARGE;
  if (widthMm >= 0.9f * smallMaxMm) return ObjClass::MEDIUM;
  return ObjClass::SMALL;
}

const char* shapeName(Shape s) {
  switch (s) {
    case Shape::ROUND: return "round";
    case Shape::ELONGATED: return "elongated";
    case Shape::IRREGULAR: return "irregular";
    default: return "unknown";
  }
}
const char* className(ObjClass c) {
  switch (c) {
    case ObjClass::SMALL: return "small";
    case ObjClass::MEDIUM: return "medium";
    case ObjClass::LARGE: return "large";
    default: return "delicate";
  }
}

void smootherReset(ObjectSmoother& s) { s.head = 0; s.n = 0; }

void smootherAdd(ObjectSmoother& s, const ObjectMeasure& m, uint32_t nowMs) {
  s.m[s.head] = m; s.t[s.head] = nowMs;
  s.head = (uint8_t)((s.head + 1) % 16);
  if (s.n < 16) s.n++;
}

ObjectMeasure smoothed(const ObjectSmoother& s, uint32_t nowMs) {
  ObjectMeasure out{}; out.shape = Shape::UNKNOWN;
  for (float& r : out.sensorRadiusMm) r = NAN;
  float w = 0, rg = 0, h = 0; int k = 0; int newest = -1; uint32_t newestAge = 0xFFFFFFFF;
  for (int i = 0; i < s.n; i++) {
    const uint32_t age = nowMs - s.t[i];
    if (age > OBJECT_SMOOTH_MS || !s.m[i].valid) continue;
    w += s.m[i].widthMm; rg += s.m[i].rangeMm; h += s.m[i].heightMm; k++;
    if (age < newestAge) { newestAge = age; newest = i; }
  }
  if (k < 2) return out;
  out = s.m[newest];
  out.widthMm = w / k; out.rangeMm = rg / k; out.heightMm = h / k;
  out.valid = true;
  return out;
}
