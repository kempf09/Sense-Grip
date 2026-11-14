#include "tof_detect.h"

uint16_t frameMinMm(const uint16_t mm[64], uint64_t mask) {
  uint16_t best = 0xFFFF;
  for (int z = 0; z < 64; z++) {
    if (!((mask >> z) & 1ULL)) continue;
    uint16_t v = mm[z];
    if (v == 0 || v >= 4000) continue;
    if (v < best) best = v;
  }
  return best;
}

bool updatePresence(PresenceTimer& t, const uint16_t minMm[3], const uint32_t frameAgeMs[3],
                    uint16_t detectMm, uint16_t holdMs, uint32_t nowMs) {
  bool sees = false;
  for (int i = 0; i < 3; i++) {
    if (frameAgeMs[i] < 500 && minMm[i] < detectMm) { sees = true; break; }
  }
  if (!sees) { t.active = false; return false; }
  if (!t.active) { t.active = true; t.sinceMs = nowMs; }
  return (uint32_t)(nowMs - t.sinceMs) >= holdMs;
}
