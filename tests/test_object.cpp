#include "check.h"
#include "object_core.h"
#include <cmath>
#include <cstring>

namespace {
const SensorCalib CAL[3] = {defaultCalib(), defaultCalib(), defaultCalib()};
const uint64_t NO_FINGERS[3] = {0, 0, 0};

void displayOf(uint8_t z, uint8_t& row, uint8_t& col) {
  for (uint8_t r = 0; r < 8; r++) for (uint8_t c = 0; c < 8; c++)
    if (zoneSourceIndex(CAL[0].orientation, r, c) == z) { row = r; col = c; return; }
}
void castBackground(uint16_t bg[3][64]) { for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) bg[s][z] = 1200; }
// Ray-cast a vertical cylinder of radius R (axis = gripper axis) between z0 and z1; zones that
// miss it see the 1200 mm background.
void castCylinder(uint16_t f[3][64], float R, float z0, float z1) {
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) {
    uint8_t row = 0, col = 0; displayOf((uint8_t)z, row, col);
    f[s][z] = 1200;
    for (float mm = 1; mm < 1200; mm += 0.5f) {
      Vec3 p;
      if (!zonePoint((uint8_t)s, row, col, mm, CAL[s], p)) continue;
      if (p.z > z1) break;
      if (horizontalRadius(p) <= R && p.z >= z0) { f[s][z] = (uint16_t)mm; break; }
    }
  }
}
ObjectMeasure measure(float R, float z0, float z1, uint8_t valid = 0x7) {
  static uint16_t f[3][64], bg[3][64];
  castBackground(bg); castCylinder(f, R, z0, z1);
  return measureObject(f, valid, bg, NO_FINGERS, CAL);
}
}  // namespace

TEST(object_cylinder_width) {
  ObjectMeasure m = measure(30, 10, 60);
  CHECK(m.valid);
  CHECK(std::fabs(m.widthMm - 60) <= 10);
  CHECK(classify(m.widthMm, 50, 90) == ObjClass::MEDIUM);
}
TEST(object_small_and_large) {
  ObjectMeasure s = measure(15, 10, 60), l = measure(50, 10, 60);
  CHECK(s.valid && l.valid);
  CHECK(classify(s.widthMm, 50, 90) == ObjClass::SMALL);
  CHECK(classify(l.widthMm, 50, 90) == ObjClass::LARGE);
}
TEST(object_class_band) {
  CHECK(classify(46, 50, 90) == ObjClass::MEDIUM);
  CHECK(classify(44, 50, 90) == ObjClass::SMALL);
  CHECK(classify(82, 50, 90) == ObjClass::LARGE);
  CHECK(classify(80, 50, 90) == ObjClass::MEDIUM);
}
TEST(object_background_margin) {
  uint16_t f[3][64], bg[3][64];
  castBackground(bg);
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) f[s][z] = bg[s][z] - 10;
  CHECK(!measureObject(f, 0x7, bg, NO_FINGERS, CAL).valid);
}
TEST(object_elongated) {
  ObjectMeasure m = measure(15, 10, 200);
  CHECK(m.valid);
  CHECK(m.heightMm >= 1.8f * m.widthMm);
  CHECK(m.shape == Shape::ELONGATED);
}
TEST(object_round_short_cylinder) {
  ObjectMeasure m = measure(40, 10, 50);
  CHECK(m.valid);
  CHECK(m.shape == Shape::ROUND);
}
TEST(object_one_sensor_offline) {
  ObjectMeasure m = measure(30, 10, 60, 0x3);
  CHECK(m.valid);
  CHECK(std::isnan(m.sensorRadiusMm[2]));
  CHECK(std::fabs(m.widthMm - 60) <= 10);
  CHECK(m.zones[2] == 0);
  CHECK(measure(30, 10, 60, 0x1).shape == Shape::UNKNOWN);
}
TEST(object_ignores_finger_zones) {
  static uint16_t f[3][64], bg[3][64];
  castBackground(bg); castCylinder(f, 30, 10, 60);
  const uint64_t fingers[3] = {~0ULL, 0, 0};
  ObjectMeasure m = measureObject(f, 0x7, bg, fingers, CAL);
  CHECK(m.zones[0] == 0);
}
TEST(object_far_points_rejected) {
  // Everything reads 400 mm (a person / wall beyond the gripper): the points lie ~250 mm from the
  // axis (rays cross the axis and leave on the far side), so none is an object point.
  uint16_t f[3][64], bg[3][64];
  castBackground(bg);
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) f[s][z] = 400;
  ObjectMeasure m = measureObject(f, 0x7, bg, NO_FINGERS, CAL);
  CHECK(!m.valid);
  CHECK_EQ((int)m.nPoints, 0);
}
TEST(object_smoothing_window) {
  ObjectSmoother sm; smootherReset(sm);
  ObjectMeasure a{}; a.valid = true; a.rangeMm = 100; a.heightMm = 40;
  a.widthMm = 50; smootherAdd(sm, a, 0);
  a.widthMm = 60; smootherAdd(sm, a, 100);
  ObjectMeasure bad{}; bad.valid = false; bad.widthMm = 500; smootherAdd(sm, bad, 150);
  a.widthMm = 70; smootherAdd(sm, a, 200);
  ObjectMeasure s = smoothed(sm, 250);
  CHECK(s.valid);
  CHECK_NEAR(s.widthMm, 60.0f, 1e-4);
  CHECK(!smoothed(sm, 450).valid);                      // only the t=200 entry is inside 300 ms
}
TEST(object_names) {
  CHECK(std::strcmp(className(ObjClass::LARGE), "large") == 0);
  CHECK(std::strcmp(shapeName(Shape::ELONGATED), "elongated") == 0);
}
