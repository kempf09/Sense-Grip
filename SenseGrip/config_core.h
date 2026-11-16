// Gripper configuration: defaults, clamping, limit validation. Pure C++.
#pragma once
#include <stdint.h>
#include "fsr_filter.h"

// Servo speed unit: 0.732 RPM (Waveshare CF library: "V=60*0.732=43.92rpm"); 61 ~ 44.7 RPM max.
constexpr uint16_t SPEED_MAX = 61;

struct GripConfig {
  int32_t openZero; int32_t closeLimit; bool limitsValid; bool closePositive;
  uint16_t approachSpeed, graspSpeed, openSpeed;        // 1..SPEED_MAX, units of 0.732 RPM
  uint16_t closeTorqueLimit, openTorqueLimit;           // 1..430, reg 44 max current, units of 6.5 mA
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
  uint8_t fsrEnabled;                                   // bit i = FSR index i used by the FSM (added in v3)
  // v5: FSR filter (FilterKind as uint8_t). Append new fields below, never above.
  uint8_t fsrFilter;                                    // 0..4, default 2 (median)
  uint8_t fsrFilterWindow;                              // 2..20 (median uses odd 3..9)
  float fsrFilterAlpha;                                 // EMA 0.05..1.0
  float fsrKalmanQ, fsrKalmanR;                         // 1e-5..10
  // v6: homing touch reference (logical position where the fingertips touch).
  int32_t touchRef;
  bool touchValid;
};

GripConfig defaultConfig();
// Size of the config blob saved by versions 1 and 2 (everything before fsrEnabled).
constexpr unsigned GRIP_CONFIG_V2_SIZE = __builtin_offsetof(GripConfig, fsrEnabled);
// Versions 3 and 4 hold the fields before fsrFilter (V4_SIZE meaningful bytes); their stored
// blob was sizeof() of that struct, i.e. padded to the struct alignment (V4_BLOB_SIZE).
constexpr unsigned GRIP_CONFIG_V4_SIZE = __builtin_offsetof(GripConfig, fsrFilter);
constexpr unsigned GRIP_CONFIG_V4_BLOB_SIZE =
    (GRIP_CONFIG_V4_SIZE + alignof(GripConfig) - 1) / alignof(GripConfig) * alignof(GripConfig);
// Version 5 held the fields before touchRef; its stored blob was padded to the struct alignment.
constexpr unsigned GRIP_CONFIG_V5_SIZE = __builtin_offsetof(GripConfig, touchRef);
constexpr unsigned GRIP_CONFIG_V5_BLOB_SIZE =
    (GRIP_CONFIG_V5_SIZE + alignof(GripConfig) - 1) / alignof(GripConfig) * alignof(GripConfig);
// The FSR filter settings as FilterParams.
FilterParams filterParamsOf(const GripConfig& c);
// Clamps every field into range; returns true if anything changed.
bool clampConfig(GripConfig& c);
// nullptr if ok, else a human-readable problem.
const char* limitsProblem(const GripConfig& c);
// After a successful Home the logical frame is the touch-reference frame again, so limits saved
// in that frame (cleared only because the turn count was lost) can be re-enabled: true when a
// touch reference exists and the saved OPEN zero / close limit are a valid pair.
bool limitsRestorable(const GripConfig& c);
