#include "tof.h"
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_MatrixLidar.h>
#include <string.h>
#include "pins.h"
#include "shared.h"
#include "tof_detect.h"
#include "servo.h"

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

constexpr uint32_t STALE_MS = 1000;   // no good frame for this long -> shown offline

// Returns 0 on success, else the failing stage (1 probe, 2 begin, 3 ranging mode).
uint8_t initSensor(uint8_t i) {
  Wire.beginTransmission(TOF_ADDR[i]);
  if (Wire.endTransmission() != 0) return 1;
  if (lidar[i]->begin() != 0) return 2;
  if (lidar[i]->setRangingMode(eMatrix_8X8) != 0) return 3;
  return 0;
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

// Same behaviour as Release 3.0.3 (which was more stable on this hardware): initialise
// once, read through errors, never re-initialise a sensor that has started. Only a sensor
// whose init never succeeded is retried (every 2 s). "online" = initialised and a good
// frame within STALE_MS; stale frames are ignored by the trigger after 500 ms (tof_detect).
void tofTask(void*) {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
  Wire.setTimeOut(120);

  bool inited[TOF_COUNT] = {false, false, false};
  bool wasOnline[TOF_COUNT] = {false, false, false};
  uint32_t lastTry[TOF_COUNT] = {0, 0, 0};
  uint32_t lastGood[TOF_COUNT] = {0, 0, 0};
  bool everGood[TOF_COUNT] = {false, false, false};
  uint32_t readMaxWin[TOF_COUNT] = {0, 0, 0};
  uint32_t winStart = millis();
  PresenceTimer timer = {0, false};

  portENTER_CRITICAL(&gMux);
  for (uint8_t i = 0; i < TOF_COUNT; i++) gSnap.s[i].minMm = 0xFFFF;
  portEXIT_CRITICAL(&gMux);

  auto tryInit = [&](uint8_t i) {
    lastTry[i] = millis();
    const uint8_t stage = initSensor(i);
    inited[i] = stage == 0;
    portENTER_CRITICAL(&gMux);
    gSnap.s[i].initialized = inited[i];
    gSnap.s[i].initFailStage = stage;
    if (stage) gSnap.s[i].initFails++;
    portEXIT_CRITICAL(&gMux);
  };
  for (uint8_t i = 0; i < TOF_COUNT; i++) { tryInit(i); vTaskDelay(1); }

  for (;;) {
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
      uint32_t now = millis();
      if (!inited[i]) {
        if (now - lastTry[i] >= RETRY_MS) tryInit(i);
      } else {
        uint16_t values[64] = {0};
        const uint32_t t0 = micros();
        const uint8_t rc = lidar[i]->getAllData(values);
        const uint32_t dt = micros() - t0;
        if (dt > readMaxWin[i]) readMaxWin[i] = dt;
        now = millis();
        uint64_t mask;
        portENTER_CRITICAL(&gMux);
        mask = gMask[i];
        portEXIT_CRITICAL(&gMux);
        const uint16_t mn = rc == 0 ? frameMinMm(values, mask) : 0xFFFF;
        portENTER_CRITICAL(&gMux);
        TofSensor& s = gSnap.s[i];
        s.lastRc = rc;
        if (rc == 0) {
          memcpy(s.mm, values, sizeof(values));
          s.minMm = mn;
          s.lastGoodMs = now;
          s.ok++;
        } else {
          s.err++;
        }
        portEXIT_CRITICAL(&gMux);
        if (rc == 0) { lastGood[i] = now; everGood[i] = true; }
      }

      // Every pass: online flags, dropout counters, presence.
      now = millis();
      const bool moving = servoGet().fb.moving;
      uint16_t mins[3]; uint32_t ages[3]; bool on[3];
      uint16_t detMm, hold;
      for (int k = 0; k < 3; k++) on[k] = inited[k] && everGood[k] && (now - lastGood[k]) < STALE_MS;
      portENTER_CRITICAL(&gMux);
      for (int k = 0; k < 3; k++) {
        TofSensor& s = gSnap.s[k];
        if (wasOnline[k] && !on[k]) { s.dropouts++; if (moving) s.dropoutsMoving++; }
        s.online = on[k];
        mins[k] = s.minMm;
        ages[k] = (inited[k] && everGood[k]) ? (now - lastGood[k]) : 0xFFFFFFFFu;
      }
      detMm = gDetectMm;
      hold = gHoldMs;
      portEXIT_CRITICAL(&gMux);
      for (int k = 0; k < 3; k++) wasOnline[k] = on[k];
      const bool present = updatePresence(timer, mins, ages, detMm, hold, now);
      uint8_t cnt = 0;
      for (int k = 0; k < 3; k++) if (on[k]) cnt++;
      if (now - winStart >= 5000) {
        portENTER_CRITICAL(&gMux);
        for (int k = 0; k < 3; k++) { gSnap.s[k].readUsMax = readMaxWin[k]; readMaxWin[k] = 0; }
        portEXIT_CRITICAL(&gMux);
        winStart = now;
      }
      portENTER_CRITICAL(&gMux);
      gSnap.objectPresent = present;
      gSnap.online = cnt;
      portEXIT_CRITICAL(&gMux);
      vTaskDelay(1);
    }
  }
}
