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
  uint32_t lastChangeMs;    // last time a good frame differed from the previous one (frozen sensor check)
  uint32_t restarts;        // manual restarts (operator button)
  uint8_t restartStage;     // result of the last manual restart: 0 ok, else the init stage that failed
};
struct TofSnapshot { TofSensor s[3]; uint8_t online; bool objectPresent; };

void tofTask(void*);
TofSnapshot tofGet();
void tofSetDetect(uint16_t mm, uint16_t holdMs, const uint64_t mask[3]);
// Operator restart (dashboard button): re-initialise the sensors in mask (bit s = sensor s). With
// busRecover the I2C bus is released first (clock pulses on SCL + STOP, Wire restarted), for a
// sensor that holds the bus. Handled by the tof task on its next pass (any task may call this).
void tofRequestRestart(uint8_t mask, bool busRecover);
