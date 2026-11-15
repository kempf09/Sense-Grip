#include "config.h"
#include <Preferences.h>
#include <math.h>

namespace {
constexpr uint8_t CFG_VERSION = 4;   // 3: + fsrEnabled; 4: same layout, older limits invalidated
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
  const size_t want = (ver == CFG_VERSION || ver == 3) ? sizeof(GripConfig)
                    : (ver == 1 || ver == 2) ? GRIP_CONFIG_V2_SIZE : 0;
  if (want && len == want) {
    GripConfig tmp = defaultConfig();          // fields missing from older blobs keep defaults
    if (p.getBytes("cfg", &tmp, want) == want) {
      upgraded = ver != CFG_VERSION;
      if (upgraded) tmp.limitsValid = false;
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
