#include "sweep_store.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>

namespace {
constexpr uint32_t MAGIC = 0x53575032;      // "SWP2": SWP1 files predate SweepModel::firstSeen and the background fix; re-run the sweep
SweepSample* gSamples = nullptr;
SweepSample* gScratch = nullptr;
SweepModel gModel = {};
bool gLoaded = false;
bool gMounted = false;
char gErr[96];
const char* fail(const char* what) {
  if (gMounted) snprintf(gErr, sizeof gErr, "%s (LittleFS %u/%u bytes used)", what,
                         (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  else snprintf(gErr, sizeof gErr, "%s (LittleFS not mounted: no data partition?)", what);
  return gErr;
}
}  // namespace

bool sweepStoreBegin() {
  if (!gSamples) {
    gSamples = (SweepSample*)ps_malloc(sizeof(SweepSample) * SWEEP_MAX);
    if (!gSamples) gSamples = (SweepSample*)malloc(sizeof(SweepSample) * SWEEP_MAX);
  }
  if (!gScratch) {
    gScratch = (SweepSample*)ps_malloc(sizeof(SweepSample) * SWEEP_MAX);
    if (!gScratch) gScratch = (SweepSample*)malloc(sizeof(SweepSample) * SWEEP_MAX);
  }
  gModel.n = 0; gLoaded = false;
  if (!gSamples) return false;
  gMounted = LittleFS.begin(true);
  if (!gMounted) return false;
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

SweepSample* sweepScratch() { return gScratch; }

const char* sweepStoreCommit(int n, const SensorCalib cal[3], int32_t openZero, int32_t touchRef, bool closePositive) {
  if (!gSamples || !gScratch) return fail("no sweep buffer");
  if (n < 20 || n > SWEEP_MAX) return fail("bad sample count");
  if (!gMounted) gMounted = LittleFS.begin(true);       // retry the mount (formats if needed)
  if (!gMounted) return fail("mount failed");
  static SweepModel model;                               // analyze into a temporary model first
  sweepAnalyze(gScratch, n, cal, openZero, touchRef, closePositive, model);
  File f = LittleFS.open("/sweep.tmp", "w");
  if (!f) return fail("cannot create /sweep.tmp");
  const uint32_t magic = MAGIC; const int32_t nn = n;
  bool ok = f.write((const uint8_t*)&magic, 4) == 4 && f.write((const uint8_t*)&nn, 4) == 4 &&
            f.write((const uint8_t*)&model, sizeof(model)) == sizeof(model) &&
            f.write((const uint8_t*)gScratch, sizeof(SweepSample) * n) == sizeof(SweepSample) * (size_t)n;
  f.close();
  if (!ok) { LittleFS.remove("/sweep.tmp"); return fail("write failed (flash full?)"); }
  LittleFS.remove("/sweep.bin");                         // old file kept until the new one is complete
  if (!LittleFS.rename("/sweep.tmp", "/sweep.bin")) return fail("rename /sweep.tmp failed");
  memcpy(gSamples, gScratch, sizeof(SweepSample) * n);   // only now replace the loaded sweep
  gModel = model;
  gLoaded = true;
  return nullptr;
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
