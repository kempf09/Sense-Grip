#include "fsr.h"
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
#include "pins.h"
#include "shared.h"

namespace {
constexpr uint8_t ADC_SAMPLES = 16;
const FsrCal DEFAULT_CAL = {2000, 1, 30000, 10, 3000, 0};

FsrCal gCal[FSR_COUNT];
bool gStored[FSR_COUNT];
FsrSnapshot gSnap = {};
ContactDebounce gDeb = {};

String key(uint8_t i, const char* suffix) { return "s" + String(i) + suffix; }

bool writeCal(uint8_t i, const FsrCal& c) {
  Preferences p;
  if (!p.begin("fsr402", false)) return false;
  bool ok = p.putFloat(key(i, "r").c_str(), c.fixedOhms) == sizeof(float) &&
            p.putFloat(key(i, "f1").c_str(), c.f1) == sizeof(float) &&
            p.putFloat(key(i, "o1").c_str(), c.o1) == sizeof(float) &&
            p.putFloat(key(i, "f2").c_str(), c.f2) == sizeof(float) &&
            p.putFloat(key(i, "o2").c_str(), c.o2) == sizeof(float) &&
            p.putFloat(key(i, "z").c_str(), c.zeroMv) == sizeof(float);
  p.end();
  return ok;
}
}  // namespace

void fsrBegin() {
  analogReadResolution(12);
  for (uint8_t i = 0; i < FSR_COUNT; i++) analogSetPinAttenuation(FSR_PINS[i], ADC_11db);

  FsrCal cal[FSR_COUNT];
  bool stored[FSR_COUNT];
  for (uint8_t i = 0; i < FSR_COUNT; i++) { cal[i] = DEFAULT_CAL; stored[i] = false; }
  Preferences p;
  if (p.begin("fsr402", true)) {
    for (uint8_t i = 0; i < FSR_COUNT; i++) {
      stored[i] = p.isKey(key(i, "r").c_str()) && p.isKey(key(i, "f1").c_str()) &&
                  p.isKey(key(i, "o1").c_str()) && p.isKey(key(i, "f2").c_str()) &&
                  p.isKey(key(i, "o2").c_str());
      cal[i].fixedOhms = p.getFloat(key(i, "r").c_str(), cal[i].fixedOhms);
      cal[i].f1 = p.getFloat(key(i, "f1").c_str(), cal[i].f1);
      cal[i].o1 = p.getFloat(key(i, "o1").c_str(), cal[i].o1);
      cal[i].f2 = p.getFloat(key(i, "f2").c_str(), cal[i].f2);
      cal[i].o2 = p.getFloat(key(i, "o2").c_str(), cal[i].o2);
      cal[i].zeroMv = p.getFloat(key(i, "z").c_str(), 0);
    }
    p.end();
  }
  portENTER_CRITICAL(&gMux);
  memcpy(gCal, cal, sizeof(cal));
  memcpy(gStored, stored, sizeof(stored));
  portEXIT_CRITICAL(&gMux);
}

void fsrSample(float contactN, uint8_t samples, uint8_t enabledMask) {
  FsrCal cal[FSR_COUNT];
  bool stored[FSR_COUNT];
  portENTER_CRITICAL(&gMux);
  memcpy(cal, gCal, sizeof(cal));
  memcpy(stored, gStored, sizeof(stored));
  portEXIT_CRITICAL(&gMux);

  FsrSnapshot next = {};
  for (uint8_t i = 0; i < FSR_COUNT; i++) {
    uint32_t sum = 0;
    for (uint8_t n = 0; n < ADC_SAMPLES; n++) sum += analogReadMilliVolts(FSR_PINS[i]);
    float mv = (float)sum / ADC_SAMPLES - cal[i].zeroMv;
    if (mv < 0) mv = 0;
    next.mv[i] = mv;
    next.raw[i] = (uint16_t)analogRead(FSR_PINS[i]);
    next.ohms[i] = fsrOhms(mv, cal[i].fixedOhms);
    next.force[i] = fsrForce(cal[i], next.ohms[i]);
    next.calibrated[i] = stored[i] && fsrCalValid(cal[i]);
  }
  // Raw readings stay visible for every channel; contact and force aggregates use enabled ones only.
  float used[FSR_COUNT];
  maskForces(next.force, enabledMask, used);
  updateContacts(gDeb, used, contactN, samples);
  next.contacts = gDeb;
  next.agg = aggregate(used, contactN);

  portENTER_CRITICAL(&gMux);
  next.seq = gSnap.seq + 1;
  gSnap = next;
  portEXIT_CRITICAL(&gMux);
}

FsrSnapshot fsrGet() { return snap(gSnap); }

FsrCal fsrGetCal(uint8_t i) {
  if (i >= FSR_COUNT) return DEFAULT_CAL;
  FsrCal c;
  portENTER_CRITICAL(&gMux);
  c = gCal[i];
  portEXIT_CRITICAL(&gMux);
  return c;
}

bool fsrSaveCal(uint8_t i, const FsrCal& c) {
  if (i >= FSR_COUNT || !fsrCalValid(c)) return false;
  if (!writeCal(i, c)) return false;
  portENTER_CRITICAL(&gMux);
  gCal[i] = c;
  gStored[i] = true;
  portEXIT_CRITICAL(&gMux);
  return true;
}

bool fsrZero(uint8_t i) {
  if (i >= FSR_COUNT) return false;
  FsrCal c;
  float mv;
  bool stored;
  portENTER_CRITICAL(&gMux);
  c = gCal[i];
  stored = gStored[i];
  mv = gSnap.mv[i];   // already net of the current zero
  portEXIT_CRITICAL(&gMux);
  if (!stored) return false;   // channel must be calibrated first; never persist defaults
  c.zeroMv += mv;
  return fsrSaveCal(i, c);   // validates (zeroMv <= 1000) before writing
}
