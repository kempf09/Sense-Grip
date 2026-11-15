#include "check.h"
#include "tof_geometry.h"
#include <cmath>

static float dist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

TEST(geom_source_index_orientations) {
  CHECK_EQ(zoneSourceIndex(0, 2, 5), (uint8_t)21);
  CHECK_EQ(zoneSourceIndex(4, 2, 5), (uint8_t)18);
  CHECK_EQ(zoneSourceIndex(2, 0, 0), (uint8_t)63);
}

TEST(geom_sensor_origin) {
  SensorCalib c = defaultCalib();
  Vec3 p;
  CHECK(!zonePoint(0, 3, 3, 0.5f, c, p));                // < 1 mm after calibration
  CHECK(zonePoint(0, 3, 3, 100, c, p));
  CHECK(horizontalRadius(p) < 101.0f);                     // looks inward
  CHECK(p.z > 0);                                          // and upward (pitched 30 deg)
}

TEST(geom_symmetry) {
  SensorCalib c = defaultCalib();
  Vec3 a, b, d;
  CHECK(zonePoint(0, 2, 6, 150, c, a));
  CHECK(zonePoint(1, 2, 6, 150, c, b));
  CHECK(zonePoint(2, 2, 6, 150, c, d));
  CHECK_NEAR(horizontalRadius(a), horizontalRadius(b), 1e-3);
  CHECK_NEAR(horizontalRadius(a), horizontalRadius(d), 1e-3);
  CHECK_NEAR(a.z, b.z, 1e-3);
  CHECK_NEAR(a.z, d.z, 1e-3);
}

TEST(geom_range_calibration) {
  SensorCalib c = defaultCalib();
  c.rangeScale = 1.1f; c.rangeOffset = -5;
  Vec3 p;
  CHECK(zonePoint(0, 4, 4, 100, c, p));
  CHECK_NEAR(dist(p, sensorOrigin(0)), 105.0f, 1e-3);
}

TEST(geom_rotation_yaw) {
  SensorCalib c = defaultCalib();
  Vec3 a, b;
  CHECK(zonePoint(1, 4, 4, 100, c, a));
  c.yawDeg = 5;
  CHECK(zonePoint(1, 4, 4, 100, c, b));
  CHECK(dist(a, b) > 1.0f);
}
