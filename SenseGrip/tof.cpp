#include "tof.h"
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_MatrixLidar.h>
#include <string.h>
#include "pins.h"
#include "shared.h"
#include "tof_detect.h"

namespace {
DFRobot_MatrixLidar_I2C lidar0(TOF_ADDR[0], &Wire);
DFRobot_MatrixLidar_I2C lidar1(TOF_ADDR[1], &Wire);
DFRobot_MatrixLidar_I2C lidar2(TOF_ADDR[2], &Wire);
DFRobot_MatrixLidar_I2C* const lidar[TOF_COUNT] = {&lidar0, &lidar1, &lidar2};

constexpr uint32_t RETRY_MS = 2000;
// Read errors in short streaks are normal on these sensors (Release 3.0.3 just counted
// them). A sensor is only re-initialised after this long without a good frame, and only
// marked offline when it also stops answering on the bus. Stale frames are ignored by the
// presence logic after 500 ms either way (tof_detect).
constexpr uint32_t STALL_MS = 5000;

bool probe(uint8_t i) {
  Wire.beginTransmission(TOF_ADDR[i]);
  return Wire.endTransmission() == 0;
}

TofSnapshot gSnap = {};
uint16_t gDetectMm = 80;
uint16_t gHoldMs = 200;
uint64_t gMask[3] = {~0ULL, ~0ULL, ~0ULL};

bool initSensor(uint8_t i) {
  Wire.beginTransmission(TOF_ADDR[i]);
  if (Wire.endTransmission() != 0) return false;
  if (lidar[i]->begin() != 0) return false;
  return lidar[i]->setRangingMode(eMatrix_8X8) == 0;
}
}  // namespace

void tofSetDetect(uint16_t mm, uint16_t holdMs, const uint64_t mask[3]) {
  portENTER_CRITICAL(&gMux);
  gDetectMm = mm;
  gHoldMs = holdMs;
  for (int i = 0; i < 3; i++) gMask[i] = mask[i];
  portEXIT_CRITICAL(&gMux);
}

TofSnapshot tofGet() { return snap(gSnap); }

void tofTask(void*) {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
  Wire.setTimeOut(120);

  bool online[TOF_COUNT] = {false, false, false};
  uint32_t lastTry[TOF_COUNT] = {0, 0, 0};
  uint32_t lastGood[TOF_COUNT] = {0, 0, 0};
  uint32_t lastProgress[TOF_COUNT] = {0, 0, 0};  // init or good frame; drives dropout
  bool everGood[TOF_COUNT] = {false, false, false};
  PresenceTimer timer = {0, false};

  portENTER_CRITICAL(&gMux);
  for (uint8_t i = 0; i < TOF_COUNT; i++) gSnap.s[i].minMm = 0xFFFF;
  portEXIT_CRITICAL(&gMux);

  // Mark sensor i offline/online; stale data is invalidated either way.
  auto setOnline = [&](uint8_t i, bool on, uint32_t now) {
    online[i] = on;
    everGood[i] = false;
    lastProgress[i] = now;
    portENTER_CRITICAL(&gMux);
    gSnap.s[i].online = on;
    gSnap.s[i].minMm = 0xFFFF;
    portEXIT_CRITICAL(&gMux);
  };

  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    uint32_t now = millis();
    lastTry[i] = now;
    setOnline(i, initSensor(i), now);
    vTaskDelay(1);
  }

  for (;;) {
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
      uint32_t now = millis();
      if (!online[i]) {
        if (now - lastTry[i] >= RETRY_MS) {
          lastTry[i] = now;
          if (initSensor(i)) setOnline(i, true, millis());
        }
      } else {
        uint16_t values[64] = {0};
        uint8_t rc = lidar[i]->getAllData(values);
        now = millis();
        uint64_t mask;
        portENTER_CRITICAL(&gMux);
        mask = gMask[i];
        portEXIT_CRITICAL(&gMux);
        uint16_t mn = rc == 0 ? frameMinMm(values, mask) : 0;

        portENTER_CRITICAL(&gMux);
        TofSensor& s = gSnap.s[i];
        if (rc == 0) {
          memcpy(s.mm, values, sizeof(values));
          s.minMm = mn;
          s.lastGoodMs = now;
          s.ok++;
        } else {
          s.err++;
        }
        portEXIT_CRITICAL(&gMux);
        if (rc == 0) {
          lastGood[i] = lastProgress[i] = now;
          everGood[i] = true;
        } else if (now - lastProgress[i] > STALL_MS) {
          lastTry[i] = now;
          if (!probe(i)) {
            setOnline(i, false, now);                 // gone from the bus: retry init every 2 s
          } else {
            lastProgress[i] = now;                    // still answering: restart ranging in place
            initSensor(i);
          }
        }
      }

      // Every pass, regardless of sensor state: refresh online count and presence.
      now = millis();
      uint16_t mins[3];
      uint32_t ages[3];
      uint16_t detMm, hold;
      portENTER_CRITICAL(&gMux);
      for (int k = 0; k < 3; k++) {
        mins[k] = gSnap.s[k].minMm;
        ages[k] = (online[k] && everGood[k]) ? (now - lastGood[k]) : 0xFFFFFFFFu;
      }
      detMm = gDetectMm;
      hold = gHoldMs;
      portEXIT_CRITICAL(&gMux);
      bool present = updatePresence(timer, mins, ages, detMm, hold, now);
      uint8_t cnt = 0;
      for (int k = 0; k < 3; k++) if (online[k]) cnt++;
      portENTER_CRITICAL(&gMux);
      gSnap.objectPresent = present;
      gSnap.online = cnt;
      portEXIT_CRITICAL(&gMux);
      vTaskDelay(1);
    }
  }
}
