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
  bool everGood[TOF_COUNT] = {false, false, false};
  PresenceTimer timer = {0, false};

  portENTER_CRITICAL(&gMux);
  for (uint8_t i = 0; i < TOF_COUNT; i++) gSnap.s[i].minMm = 0xFFFF;
  portEXIT_CRITICAL(&gMux);
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    online[i] = initSensor(i);
    lastTry[i] = lastGood[i] = millis();
    portENTER_CRITICAL(&gMux);
    gSnap.s[i].online = online[i];
    portEXIT_CRITICAL(&gMux);
    vTaskDelay(1);
  }

  for (;;) {
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
      uint32_t now = millis();
      if (!online[i]) {
        if (now - lastTry[i] >= RETRY_MS) {
          lastTry[i] = now;
          online[i] = initSensor(i);
          lastGood[i] = millis();
          portENTER_CRITICAL(&gMux);
          gSnap.s[i].online = online[i];
          portEXIT_CRITICAL(&gMux);
        }
        vTaskDelay(1);
        continue;
      }

      uint16_t values[64] = {0};
      uint8_t rc = lidar[i]->getAllData(values);
      now = millis();
      uint64_t mask;
      portENTER_CRITICAL(&gMux);
      mask = gMask[i];
      portEXIT_CRITICAL(&gMux);

      portENTER_CRITICAL(&gMux);
      TofSensor& s = gSnap.s[i];
      if (rc == 0) {
        memcpy(s.mm, values, sizeof(values));
        s.minMm = frameMinMm(values, mask);
        s.lastGoodMs = now;
        s.ok++;
        lastGood[i] = now;
        everGood[i] = true;
      } else {
        s.err++;
      }
      portEXIT_CRITICAL(&gMux);

      // Presence over all sensors, using the current detect settings.
      uint16_t mins[3];
      uint32_t ages[3];
      uint16_t detMm, hold;
      portENTER_CRITICAL(&gMux);
      for (int k = 0; k < 3; k++) {
        mins[k] = gSnap.s[k].minMm;
        ages[k] = everGood[k] ? (now - lastGood[k]) : 0xFFFFFFFFu;
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

      // Repeated failures: drop to offline so init retry kicks in.
      if (rc != 0 && now - lastGood[i] > 1000) {
        online[i] = false;
        lastTry[i] = now;
        portENTER_CRITICAL(&gMux);
        gSnap.s[i].online = false;
        portEXIT_CRITICAL(&gMux);
      }
      vTaskDelay(1);
    }
  }
}
