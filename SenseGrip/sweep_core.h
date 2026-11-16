// Calibration-sweep analysis: which SEN0628 zones see the fingers, fingertip opening vs motor
// position, and the position estimate / consistency check used after power-up. Pure C++.
#pragma once
#include <stdint.h>
#include "tof_geometry.h"

constexpr int SWEEP_MAX = 260;                         // ~20 000 counts / 100 + margin
constexpr float OPENING_AT_OPEN_MM = 170.28f;          // fingertip radius at OPEN zero (3.0.3 geometry)
constexpr int32_t FINGERS_INVISIBLE_COUNTS = 1000;     // fingers not seen this close to OPEN zero

struct SweepSample { int32_t pos; uint16_t mm[3][64]; uint8_t valid; /* bit s: sensor s frame present */ };
struct SweepModel {
  int n; int32_t openZero, touchRef;
  uint64_t fingerZones[3];                             // bit z (raw index) set: zone shows a finger
  float opening[SWEEP_MAX];                            // fingertip radius (mm) at sample i
  int firstSeen;                                       // first sample with a measured fingertip (-1: none); earlier ones are extrapolated
};
struct PosEstimate { bool valid; int32_t pos; float score; };

// n >= 20, first sample within 150 counts of openZero, positions strictly monotonic toward closing.
bool sweepValid(const SweepSample* s, int n, int32_t openZero, bool closePositive);
// Same check; nullptr when valid, else the reason (written into buf).
const char* sweepInvalidWhy(const SweepSample* s, int n, int32_t openZero, bool closePositive, char* buf, int bufLen);
void  sweepAnalyze(const SweepSample* s, int n, const SensorCalib cal[3], int32_t openZero,
                   int32_t touchRef, bool closePositive, SweepModel& out);
// Robust smoothing of a measured opening curve in place: median of 5 (symmetric window, shrinking at the ends),
// then the closest non-increasing sequence (pool-adjacent-violators). The fingertips only close.
void smoothOpening(float* o, int n);
// Fingertip opening (mm) at a logical position: linear interpolation between samples.
float openingAt(const SweepModel& m, const SweepSample* s, int32_t pos);
// Best-matching sweep sample for a live frame (finger zones only).
PosEstimate estimatePosition(const SweepModel& m, const SweepSample* s, const uint16_t frame[3][64], uint8_t frameValid);
// True when the live frame disagrees with the sample nearest pos by more than thresholdMm (RMS
// over finger zones); false (suspended) when fewer than 6 zones can be compared.
// Something besides the fingers is in view: >= 3 non-finger zones read at least 50 mm closer than
// the sweep recorded at this position (or read a target where the sweep saw none). The position
// check is suspended then, since such an object can also cover finger zones.
bool  viewObstructed(const SweepModel& m, const SweepSample* s, int32_t pos, const uint16_t frame[3][64], uint8_t frameValid);
bool  positionMismatch(const SweepModel& m, const SweepSample* s, int32_t pos, const uint16_t frame[3][64],
                       uint8_t frameValid, float thresholdMm);
