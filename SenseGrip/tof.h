// SEN0628 8x8 ToF acquisition. Only this module touches I2C (Wire).
#pragma once
#include <stdint.h>

struct TofSensor { bool online; uint16_t mm[64]; uint16_t minMm; uint32_t lastGoodMs, ok, err; };
struct TofSnapshot { TofSensor s[3]; uint8_t online; bool objectPresent; };

void tofTask(void*);
TofSnapshot tofGet();
void tofSetDetect(uint16_t mm, uint16_t holdMs, const uint64_t mask[3]);
