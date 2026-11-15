// FSR acquisition and calibration storage. Only this module touches the ADC.
#pragma once
#include <stdint.h>
#include "fsr_math.h"
#include "fsr_filter.h"

struct FsrSnapshot {
  uint16_t raw[6]; float mv[6], ohms[6]; bool calibrated[6];
  float forceRaw[6];   // calibrated force before filtering
  float force[6];      // filtered force (used by the FSM)
  float limitForce;    // force-limit input: max over enabled channels of max(raw, filtered)
  ContactDebounce contacts; FsrAggregate agg; uint32_t seq;
};

void fsrBegin();                                   // ADC setup + load calibration from NVS
// Control task, every 10 ms; bit i = FSR i used. Filter states reset when fp changes.
void fsrSample(float contactN, uint8_t samples, uint8_t enabledMask, const FilterParams& fp);
FsrSnapshot fsrGet();
FsrCal fsrGetCal(uint8_t i);
bool fsrSaveCal(uint8_t i, const FsrCal& c);
bool fsrZero(uint8_t i);                           // zeroMv += current mv, saved
