// SEN0628 8x8 ToF acquisition. Only this module touches I2C (Wire).
#pragma once
#include <stdint.h>

struct TofSensor {
  bool online;              // initialised AND a good frame within the last second
  bool initialized;         // init succeeded once; never torn down afterwards
  uint16_t mm[64]; uint16_t minMm; uint32_t lastGoodMs, ok, err;
  // Diagnostics (shown on the SEN0628 tab)
  uint8_t initFailStage;    // last init failure: 0 none, 1 no I2C ACK, 2 begin(), 3 setRangingMode()
  uint32_t initFails;       // init attempts that failed
  uint8_t lastRc;           // last getAllData() return code (0 = ok)
  uint32_t readUsMax;       // longest getAllData() in the last 5 s
  uint32_t dropouts;        // online -> stale transitions
  uint32_t dropoutsMoving;  // ... of which happened while the servo was moving
};
struct TofSnapshot { TofSensor s[3]; uint8_t online; bool objectPresent; };

void tofTask(void*);
TofSnapshot tofGet();
void tofSetDetect(uint16_t mm, uint16_t holdMs, const uint64_t mask[3]);
