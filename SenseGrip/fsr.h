// FSR acquisition and calibration storage. Only this module touches the ADC.
#pragma once
#include <stdint.h>
#include "fsr_math.h"

struct FsrSnapshot {
  uint16_t raw[6]; float mv[6], ohms[6], force[6]; bool calibrated[6];
  ContactDebounce contacts; FsrAggregate agg; uint32_t seq;
};

void fsrBegin();                                   // ADC setup + load calibration from NVS
void fsrSample(float contactN, uint8_t samples);   // control task, every 20 ms
FsrSnapshot fsrGet();
FsrCal fsrGetCal(uint8_t i);
bool fsrSaveCal(uint8_t i, const FsrCal& c);
bool fsrZero(uint8_t i);                           // zeroMv += current mv, saved
