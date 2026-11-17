#include "config.h"
#include <string.h>
#include <Preferences.h>
#include <math.h>

namespace {
constexpr uint8_t CFG_VERSION = 9;   // 3: + fsrEnabled; 4: older limits invalidated; 5: + FSR filter; 6: + touchRef; 7: + class profiles; 8: + sweep settings; 9: + slip detection
// Limits saved before v4 were taken while the servo's position count could be reset by a
// mode write (hardware-observed), so they may not match the jaw: v1..v3 blobs load with
// limitsValid = false. (v2 builds saved speeds up to 3400; clampConfig brings them to <= 61.)

// Release 3.0.3 FsmConfig layout, copied verbatim for migration (read-only).
struct FsmConfig {
  uint32_t magic;
  bool closePositive,autoStart,openZeroValid;
  int32_t openZero;
  uint16_t idleDetectMm,positionTargetMm,positionToleranceMm,maxFingerDeviationMm;
  float graspForceN,fsrSlipDropPct,contactForceN,maxForceN;
  uint16_t minimumActiveFsr,contactMinActiveFsr,graspSpeed,positionSpeed,releaseSpeed,closeCurrentLimit,openCurrentLimit,stallCurrentMa;
  uint8_t stallPercent,slipDeltaMm,slipConsecutive,maxRetries,contactConsecutive,forceConsecutive;
  uint16_t openPositionToleranceCounts;
  uint32_t positioningTimeoutMs,graspTimeoutMs,releaseTimeoutMs,stallDebounceMs;
  int32_t touchPosition;
  bool touchPositionValid;
  uint16_t travelSafetyMarginCounts,contactStepCounts,graspStepCounts,manualJogStepCounts;
};
constexpr uint32_t FSM_MAGIC = 0x46534D34;

// Returns true when sgcfg held a usable blob; sets upgraded when it was an older version
// (v1/v2 blobs are the v3 struct without the trailing fsrEnabled byte).
bool loadNew(GripConfig& out, bool& upgraded) {
  upgraded = false;
  Preferences p;
  if (!p.begin("sgcfg", true)) return false;
  bool ok = false;
  const uint8_t ver = p.getUChar("ver", 0);
  const size_t len = p.getBytesLength("cfg");
  // stored = blob length on flash; keep = meaningful prefix copied over the defaults
  // (older blobs end in padding that must not overwrite newer fields).
  const size_t stored = ver == CFG_VERSION ? sizeof(GripConfig)
                      : ver == 8 ? GRIP_CONFIG_V8_BLOB_SIZE
                      : ver == 7 ? GRIP_CONFIG_V7_BLOB_SIZE
                      : ver == 6 ? GRIP_CONFIG_V6_BLOB_SIZE
                      : ver == 5 ? GRIP_CONFIG_V5_BLOB_SIZE
                      : (ver == 3 || ver == 4) ? GRIP_CONFIG_V4_BLOB_SIZE
                      : (ver == 1 || ver == 2) ? GRIP_CONFIG_V2_SIZE : 0;
  const size_t keep = ver == CFG_VERSION ? sizeof(GripConfig)
                    : ver == 8 ? GRIP_CONFIG_V8_SIZE
                    : ver == 7 ? GRIP_CONFIG_V7_SIZE
                    : ver == 6 ? GRIP_CONFIG_V6_SIZE
                    : ver == 5 ? GRIP_CONFIG_V5_SIZE
                    : (ver == 3 || ver == 4) ? GRIP_CONFIG_V4_SIZE : GRIP_CONFIG_V2_SIZE;
  if (stored && len == stored && stored <= sizeof(GripConfig) + 8) {
    GripConfig tmp = defaultConfig();          // fields missing from older blobs keep defaults
    uint8_t raw[sizeof(GripConfig) + 8];
    if (p.getBytes("cfg", raw, stored) == stored) {
      memcpy(&tmp, raw, keep);
      upgraded = ver != CFG_VERSION;
      if (ver <= 3) tmp.limitsValid = false;   // v4+ limits were taken in the stable frame
      upgradeConfig(tmp, ver);                  // e.g. < v7: Max force 3 N -> 8 N
      out = tmp; ok = true;
    }
  }
  p.end();
  return ok;
}

bool loadLegacy(GripConfig& out) {
  Preferences p;
  if (!p.begin("fsm", true)) return false;   // read-only: never modifies legacy data
  bool ok = false;
  if (p.getBytesLength("cfg") == sizeof(FsmConfig)) {
    FsmConfig f;
    if (p.getBytes("cfg", &f, sizeof(f)) == sizeof(f) && f.magic == FSM_MAGIC) {
      GripConfig c = defaultConfig();
      c.openZero = f.openZero;
      c.closePositive = f.closePositive;
      int32_t sign = f.closePositive ? 1 : -1;
      c.closeLimit = f.touchPosition - sign * (int32_t)f.travelSafetyMarginCounts;
      c.limitsValid = false;   // the position frame may have changed since (see CFG_VERSION note)
      out = c;
      ok = true;
    }
  }
  p.end();
  return ok;
}
}  // namespace

bool configSave(const GripConfig& c) {
  Preferences p;
  if (!p.begin("sgcfg", false)) return false;
  bool ok = p.putBytes("cfg", &c, sizeof(c)) == sizeof(c) &&
            p.putUChar("ver", CFG_VERSION) == sizeof(uint8_t);
  p.end();
  return ok;
}

GripConfig configLoad() {
  GripConfig c = defaultConfig();
  bool upgraded = false;
  if (loadNew(c, upgraded)) {
    if (upgraded) { clampConfig(c); configSave(c); }   // older blob: limits invalidated, rest kept
  } else {
    c = defaultConfig();
    if (loadLegacy(c)) {
      clampConfig(c);
      configSave(c);   // one-time write to sgcfg only
    }
  }
  clampConfig(c);
  return c;
}

void servoIdBaudLoad(uint8_t& id, uint32_t& baud) {
  id = 1; baud = 1000000;
  Preferences p;
  if (p.begin("cf35", true)) {
    id = p.getUChar("id", 1);
    baud = p.getULong("baud", 1000000);
    p.end();
  }
}

bool servoIdBaudSave(uint8_t id, uint32_t baud) {
  Preferences p;
  if (!p.begin("cf35", false)) return false;
  bool ok = p.putUChar("id", id) == sizeof(uint8_t) &&
            p.putULong("baud", baud) == sizeof(uint32_t);
  p.end();
  return ok;
}
