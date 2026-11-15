#include "tof_geometry.h"
#include <cmath>

// Release 3.0.3: CFG={radius:101.0,z:0.0,pitchDeg:30,yawDeg:[90,210,330],zoneStepDeg:7.5,...}
namespace {
constexpr float RADIUS = 101.0f, Z0 = 0.0f, PITCH_DEG = 30.0f, ZONE_STEP_DEG = 7.5f;
constexpr float YAW_DEG[3] = {90.0f, 210.0f, 330.0f};
constexpr float DEG = 3.14159265358979f / 180.0f;

Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 mul(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
Vec3 norm(Vec3 a) { const float n = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); return n > 0 ? mul(a, 1 / n) : a; }

struct Axes { Vec3 right, down, forward, origin; };
Axes mechanicalAxes(uint8_t s) {                       // JS mechanicalAxes
  const float a = YAW_DEG[s] * DEG, b = PITCH_DEG * DEG;
  const Vec3 er = {std::cos(a), std::sin(a), 0}, et = {-std::sin(a), std::cos(a), 0}, up = {0, 0, 1};
  return {et,
          add(mul(er, -std::sin(b)), mul(up, -std::cos(b))),
          add(mul(er, -std::cos(b)), mul(up, std::sin(b))),
          {RADIUS * std::cos(a), RADIUS * std::sin(a), Z0}};
}
Vec3 localRay(float row, float col) {                  // JS localRay
  const float ax = (col - 3.5f) * ZONE_STEP_DEG * DEG, ay = (3.5f - row) * ZONE_STEP_DEG * DEG, ca = std::cos(ay);
  return norm({ca * std::sin(ax), -std::sin(ay), ca * std::cos(ax)});
}
Vec3 rotateLocal(Vec3 v, const SensorCalib& p) {       // JS rotateLocal: roll (z), pitch (x), yaw (y)
  const float y = p.yawDeg * DEG, x = p.pitchDeg * DEG, z = p.rollDeg * DEG;
  Vec3 q = {v.x * std::cos(z) - v.y * std::sin(z), v.x * std::sin(z) + v.y * std::cos(z), v.z};
  q = {q.x, q.y * std::cos(x) - q.z * std::sin(x), q.y * std::sin(x) + q.z * std::cos(x)};
  q = {q.x * std::cos(y) + q.z * std::sin(y), q.y, -q.x * std::sin(y) + q.z * std::cos(y)};
  return q;
}
Vec3 localToWorld(uint8_t s, Vec3 v, const SensorCalib& p) {   // JS localToWorld
  const Axes a = mechanicalAxes(s);
  const Vec3 q = rotateLocal(v, p);
  return add(add(mul(a.right, q.x), mul(a.down, q.y)), mul(a.forward, q.z));
}
}  // namespace

SensorCalib defaultCalib() { return SensorCalib{4, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, false}; }

uint8_t zoneSourceIndex(uint8_t o, uint8_t row, uint8_t col) {   // JS matrixSourceIndex
  int r = row, c = col;
  switch (o) {
    case 1: r = 7 - col; c = row; break;
    case 2: r = 7 - row; c = 7 - col; break;
    case 3: r = col; c = 7 - row; break;
    case 4: r = row; c = 7 - col; break;
    case 5: r = 7 - row; c = col; break;
    case 6: r = col; c = row; break;
    case 7: r = 7 - col; c = 7 - row; break;
    default: break;
  }
  return (uint8_t)(r * 8 + c);
}

Vec3 sensorOrigin(uint8_t s) { return mechanicalAxes(s).origin; }

bool zonePoint(uint8_t s, uint8_t row, uint8_t col, float mm, const SensorCalib& c, Vec3& out) {
  const float d = mm * c.rangeScale + c.rangeOffset;     // JS pointFor
  if (!std::isfinite(d) || d < 1) return false;
  out = add(sensorOrigin(s), localToWorld(s, mul(localRay(row, col), d), c));
  return true;
}

float horizontalRadius(const Vec3& p) { return std::sqrt(p.x * p.x + p.y * p.y); }
