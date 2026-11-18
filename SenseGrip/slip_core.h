// Slip detection in MONITOR (phase 4b, docs/superpowers/specs/2026-11-17-phase4b-slip-detection-design.md).
// Pure C++: the firmware and the offline replay tool (tools/slip_replay) run this same code.
#pragma once
#include <stdint.h>

struct SlipParams {
  float fsrDropFrac;        // relative pad drop within the window that counts (0.30)
  uint16_t fsrWindowMs;     // drop window (300 ms, max 600)
  float fsrMinForceN;       // pads below this at baseline are ignored (0.15 N)
  uint16_t fsrConfirmMs;    // drop must persist (30 ms)
  uint16_t tofNearMm;       // only zones nearer than this at baseline (350 mm)
  uint16_t tofZoneMm;       // zone change threshold, mm (12) ...
  float tofZoneFrac;        // ... or this fraction of the baseline reading (0.10), whichever is larger
  uint8_t tofZoneCount;     // changed zones needed (4)
  uint16_t tofConfirmMs;    // count must hold this long (150 ms); frame records arrive per sensor
  float tofDispMm;          // smoothed-measurement displacement that counts; 0 = off (default)
  uint8_t mode;             // 0 FSR only, 1 ToF only, 2 either, 3 both within fuseWindowMs
  uint16_t fuseWindowMs;    // (500)
  uint16_t settleMs;        // after MONITOR entry before the baseline (500)
  uint16_t holdoffMs;       // after an event (1500)
};
SlipParams slipDefaults();
// Parameters by name (replay tool, config): returns false for an unknown name.
const char* slipParamName(int i);              // nullptr after the last
bool slipParamSet(SlipParams& p, const char* name, float v);
float slipParamGet(const SlipParams& p, const char* name);

struct SlipInput {
  uint32_t tMs;
  bool active;                  // FSM in MONITOR
  float force[6];               // filtered force per pad (N), any fixed order; NaN = unused
  bool frameNew;                // a new SEN0628 frame set since the previous step
  const uint16_t (*frame)[64];  // latest frame set (mm, raw zone index)
  uint8_t frameValid;           // bit s: sensor s online
  bool objValid; float rangeMm, heightMm, rad[3];   // smoothed object measurement (NaN = none)
};
enum : uint8_t { SLIP_FSR = 1, SLIP_TOF = 2 };
struct SlipOut {
  bool armed;          // baseline taken (MONITOR + settle)
  float fsrDrop;       // largest relative pad drop now
  bool fsrFlag;
  int tofZones;        // zones changed vs baseline at the latest frame set
  float tofDispMm;     // smoothed-measurement displacement (NaN when not available)
  bool tofFlag;
  bool slip;           // an event fires on this step
  uint8_t reason;      // SLIP_FSR | SLIP_TOF of this step's flags
  bool lost;           // object gone (no pad loaded at the re-baseline after an event): no more events this hold
};

constexpr int SLIP_HIST = 64;    // force history entries (10 ms steps -> 640 ms)
struct SlipState {
  bool wasActive, armed, haveFrame, baseFrame;
  uint32_t activeSince, holdUntil, fsrSince, lastFsrMs, lastTofMs;
  bool fsrOver, haveFsrFlag, haveTofFlag, holdActive;
  float base[6];
  uint32_t histT[SLIP_HIST]; float hist[SLIP_HIST][6]; int histHead, histN;
  uint16_t frame0[3][64]; uint8_t valid0;
  bool tofOver; uint32_t tofSince; int tofZones;
  bool rebasePending; uint32_t rebaseAt;     // re-baseline when the holdoff after an event ends
  bool lost;                                 // object gone: detector silent until the next hold
  bool obj0; float range0, height0, rad0[3];
};
void slipReset(SlipState& s);
SlipOut slipStep(SlipState& s, const SlipParams& p, const SlipInput& in);
