// Object perception from the three SEN0628 frames (spec §7.1/7.2): object points, width, size
// class, coarse shape, and 300 ms smoothing. Pure C++.
#pragma once
#include <stdint.h>
#include "config_core.h"
#include "tof_geometry.h"

enum class Shape : uint8_t { UNKNOWN, ROUND, ELONGATED, IRREGULAR };

struct ObjectMeasure {
  bool valid;               // >= 3 object points in total
  float widthMm;            // 2 x mean over sensors of the per-sensor median point radius
  float rangeMm;            // median sensor range (mm) of all object points
  float sensorRadiusMm[3];  // NAN where the sensor has < 2 points
  float heightMm;           // max z - min z of all object points
  uint8_t nPoints;
  uint64_t zones[3];        // raw zone bits that are object points
  Shape shape;
};

// Object point: not a finger zone, reading 1..3999, below the background by more than
// max(15 mm, 20 % of it) (or no background), within 100 mm of the axis and above the palm (z > 5).
// background: empty-gripper reading per zone at this position (0 = none).
ObjectMeasure measureObject(const uint16_t frame[3][64], uint8_t frameValid,
                            const uint16_t background[3][64], const uint64_t fingerZones[3],
                            const SensorCalib cal[3]);
// Size class with the 10 % band (an object within 10 % below a boundary takes the larger class).
// Never returns DELICATE.
ObjClass classify(float widthMm, uint16_t smallMaxMm, uint16_t mediumMaxMm);
const char* shapeName(Shape s);
const char* className(ObjClass c);

// Spec §7.2 smoothing: mean width/range/height of the valid measures of the last 300 ms (ring of
// 16 timestamped entries); shape, zones and per-sensor radii from the newest valid one.
// valid when >= 2 valid measures fall inside the window.
constexpr uint32_t OBJECT_SMOOTH_MS = 300;
struct ObjectSmoother { ObjectMeasure m[16]; uint32_t t[16]; uint8_t head, n; };
void smootherReset(ObjectSmoother& s);
void smootherAdd(ObjectSmoother& s, const ObjectMeasure& m, uint32_t nowMs);
ObjectMeasure smoothed(const ObjectSmoother& s, uint32_t nowMs);
