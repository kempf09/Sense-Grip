// Calibration-sweep storage (LittleFS /sweep.bin, samples in PSRAM) and the SEN0628 per-sensor
// calibration saved by the Release 3.0.3 3D page (NVS "senCal2").
#pragma once
#include <stdint.h>
#include "sweep_core.h"

bool sweepStoreBegin();                     // mount LittleFS, allocate the sample buffer, load /sweep.bin
SweepSample* sweepSamples();                // SWEEP_MAX entries (nullptr if allocation failed)
SweepModel& sweepModel();                   // n == 0 when no sweep is loaded
bool sweepLoaded();
bool sweepStoreSave(int n);                 // writes sweepSamples()[0..n) and sweepModel()
bool tofCalibLoad(SensorCalib out[3]);      // false (defaults returned) when senCal2 is missing
