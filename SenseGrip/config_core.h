// Gripper configuration: defaults, clamping, limit validation. Pure C++.
#pragma once
#include <stdint.h>

// Servo speed unit is steps/s (4096 steps per revolution). 3400 steps/s ~ 50 RPM.
constexpr uint16_t SPEED_MAX = 3400;

struct GripConfig {
  int32_t openZero; int32_t closeLimit; bool limitsValid; bool closePositive;
  uint16_t approachSpeed, graspSpeed, openSpeed;        // 1..SPEED_MAX, servo steps/s (4096 steps = 1 rev)
  uint16_t closeTorqueLimit, openTorqueLimit;           // 1..430
  uint16_t openToleranceCounts;                         // 1..200
  uint16_t limitSlackCounts;                            // 0..2000
  float contactForceN; uint8_t contactSamples;          // 0.05..5 / 1..20
  float graspForceN;   uint8_t graspSamples;            // 0.1..30 / 1..20
  uint8_t minActiveFsr;                                 // 1..6
  float maxForceN;                                      // 0.5..30
  uint16_t detectMm, detectHoldMs;                      // 20..1000 / 0..2000
  uint32_t preTouchTimeoutMs, graspTimeoutMs, openTimeoutMs; // 500..60000
  bool runViaPosition;
  uint64_t tofMask[3];                                  // bit z = zone z enabled
};

GripConfig defaultConfig();
// Config blobs saved with version 1 used speeds 1..61 (a wrong unit assumption,
// ~1 RPM). Replaces those speeds with the current defaults; everything else is kept.
void upgradeFromV1(GripConfig& c);
// Clamps every field into range; returns true if anything changed.
bool clampConfig(GripConfig& c);
// nullptr if ok, else a human-readable problem.
const char* limitsProblem(const GripConfig& c);
