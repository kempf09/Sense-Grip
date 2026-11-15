// SEN0628 zone geometry, ported from the Release 3.0.3 3D page (JS) to C++. Pure.
// World frame: origin at the palm centre, z up along the gripper axis.
#pragma once
#include <stdint.h>

struct Vec3 { float x, y, z; };
struct SensorCalib { uint8_t orientation; float rangeScale, rangeOffset, yawDeg, pitchDeg, rollDeg; bool valid; };

SensorCalib defaultCalib();                                  // {4, 1, 0, 0, 0, 0, false}
// Index into the sensor's 64 raw values for display row/col under an orientation (JS matrixSourceIndex).
uint8_t zoneSourceIndex(uint8_t orientation, uint8_t row, uint8_t col);
Vec3    sensorOrigin(uint8_t sensor);
// World point for display zone (row, col) of sensor at raw distance mm; false if the calibrated
// distance is < 1 mm.
bool    zonePoint(uint8_t sensor, uint8_t row, uint8_t col, float mm, const SensorCalib& c, Vec3& out);
float   horizontalRadius(const Vec3& p);
