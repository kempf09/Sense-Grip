#include "sweep_store.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>

namespace {
constexpr uint32_t MAGIC = 0x53575031;      // "SWP1"
SweepSample* gSamples = nullptr;
SweepModel gModel = {};
bool gLoaded = false;
}  // namespace

bool sweepStoreBegin() {
  if (!gSamples) {
    gSamples = (SweepSample*)ps_malloc(sizeof(SweepSample) * SWEEP_MAX);
    if (!gSamples) gSamples = (SweepSample*)malloc(sizeof(SweepSample) * SWEEP_MAX);
  }
  gModel.n = 0; gLoaded = false;
  if (!gSamples || !LittleFS.begin(true)) return false;
  File f = LittleFS.open("/sweep.bin", "r");
  if (!f) return false;
  uint32_t magic = 0; int32_t n = 0;
  bool ok = f.read((uint8_t*)&magic, 4) == 4 && magic == MAGIC &&
            f.read((uint8_t*)&n, 4) == 4 && n >= 20 && n <= SWEEP_MAX &&
            f.read((uint8_t*)&gModel, sizeof(gModel)) == sizeof(gModel) &&
            f.read((uint8_t*)gSamples, sizeof(SweepSample) * n) == sizeof(SweepSample) * (size_t)n &&
            gModel.n == n;
  f.close();
  if (!ok) { gModel.n = 0; return false; }
  gLoaded = true;
  return true;
}

SweepSample* sweepSamples() { return gSamples; }
SweepModel& sweepModel() { return gModel; }
bool sweepLoaded() { return gLoaded && gModel.n > 0; }

bool sweepStoreSave(int n) {
  if (!gSamples || n < 20 || n > SWEEP_MAX) return false;
  File f = LittleFS.open("/sweep.tmp", "w");
  if (!f) return false;
  const uint32_t magic = MAGIC; const int32_t nn = n;
  bool ok = f.write((const uint8_t*)&magic, 4) == 4 && f.write((const uint8_t*)&nn, 4) == 4 &&
            f.write((const uint8_t*)&gModel, sizeof(gModel)) == sizeof(gModel) &&
            f.write((const uint8_t*)gSamples, sizeof(SweepSample) * n) == sizeof(SweepSample) * (size_t)n;
  f.close();
  if (!ok) { LittleFS.remove("/sweep.tmp"); return false; }
  LittleFS.remove("/sweep.bin");
  if (!LittleFS.rename("/sweep.tmp", "/sweep.bin")) return false;   // old sweep kept until here
  gLoaded = true;
  return true;
}

bool tofCalibLoad(SensorCalib out[3]) {
  for (int i = 0; i < 3; i++) out[i] = defaultCalib();
  Preferences p;
  if (!p.begin("senCal2", true)) return false;
  const bool ok = p.getUChar("ver", 0) == 1;
  if (ok) {
    for (int i = 0; i < 3; i++) {
      const String k(i);
      SensorCalib c;
      c.orientation = p.getUChar(("o" + k).c_str(), 4);
      c.rangeScale = p.getFloat(("s" + k).c_str(), 1.0f);
      c.rangeOffset = p.getFloat(("b" + k).c_str(), 0.0f);
      c.yawDeg = p.getFloat(("y" + k).c_str(), 0.0f);
      c.pitchDeg = p.getFloat(("p" + k).c_str(), 0.0f);
      c.rollDeg = p.getFloat(("r" + k).c_str(), 0.0f);
      c.valid = p.getBool(("v" + k).c_str(), false);
      // Same limits as Release 3.0.3; out-of-range entries fall back to defaults.
      if (c.orientation > 7 || !(c.rangeScale >= 0.8f && c.rangeScale <= 1.2f) || !(fabsf(c.rangeOffset) <= 40) ||
          !(fabsf(c.yawDeg) <= 8) || !(fabsf(c.pitchDeg) <= 8) || !(fabsf(c.rollDeg) <= 8)) c = defaultCalib();
      out[i] = c;
    }
  }
  p.end();
  return ok;
}
