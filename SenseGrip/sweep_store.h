// Calibration-sweep storage (LittleFS /sweep.bin, samples in PSRAM) and the SEN0628 per-sensor
// calibration saved by the Release 3.0.3 3D page (NVS "senCal2").
#pragma once
#include <stdint.h>
#include "sweep_core.h"

bool sweepStoreBegin();                     // mount LittleFS, allocate the sample buffer, load /sweep.bin
SweepSample* sweepSamples();                // SWEEP_MAX entries (nullptr if allocation failed)
SweepModel& sweepModel();                   // n == 0 when no sweep is loaded
bool sweepLoaded();
SweepSample* sweepScratch();                // SWEEP_MAX entries for a sweep in progress
// Analyze scratch[0..n), save it to flash, and only then make it the loaded sweep. On failure the
// loaded sweep (RAM and flash) is unchanged.
// Returns nullptr on success, else which step failed (incl. LittleFS used/total bytes).
const char* sweepStoreCommit(int n, const SensorCalib cal[3], int32_t openZero, int32_t touchRef, bool closePositive);
bool tofCalibLoad(SensorCalib out[3]);
// Capture-background correction (spec §7.1): per-zone difference between the present empty-gripper
// view and the sweep, added to the sweep's background. RAM (zero by default), LittleFS /bgdelta.bin,
// loaded by sweepStoreBegin, cleared when a new sweep is committed.
const int16_t (*bgDelta())[64];
bool bgDeltaSave(const int16_t d[3][64]);      // false (defaults returned) when senCal2 is missing
