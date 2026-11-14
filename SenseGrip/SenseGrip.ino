#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "control.h"
#include "fsr.h"
#include "servo.h"
#include "tof.h"
#include "web.h"

void setup() {
  Serial.begin(115200);
  uint8_t id; uint32_t baud;
  servoIdBaudLoad(id, baud);
  servoBegin(id, baud);
  fsrBegin();
  controlBegin();
  WiFi.softAP("SENSE-GRIP", "12341234");
  webBegin();
  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(tofTask, "tof", 8192, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(webTask, "web", 12288, nullptr, 1, nullptr, 0);
}

void loop() { vTaskDelay(1000); }
