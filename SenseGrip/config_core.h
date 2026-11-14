// Gripper configuration: defaults, clamping, limit validation. Pure C++.
#pragma once
#include <stdint.h>

struct GripConfig {
  int32_t openZero; int32_t closeLimit; bool limitsValid; bool closePositive;
  uint16_t approachSpeed, graspSpeed, openSpeed;        // 1..61
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
// Clamps every field into range; returns true if anything changed.
bool clampConfig(GripConfig& c);
// nullptr if ok, else a human-readable problem.
const char* limitsProblem(const GripConfig& c);
