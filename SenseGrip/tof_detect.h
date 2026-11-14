#pragma once
#include <stdint.h>

// Minimum valid distance over enabled zones (mask bit z = zone z enabled).
// Ignores 0 and >=4000. Returns 0xFFFF if no valid zone.
uint16_t frameMinMm(const uint16_t mm[64], uint64_t mask);

struct PresenceTimer { uint32_t sinceMs; bool active; };

// minMm[i] = frameMinMm per sensor, frameAgeMs[i] = age of that sensor's last good frame.
// A sensor sees an object if frameAgeMs < 500 and minMm < detectMm.
// Returns true when an object has been seen continuously for >= holdMs.
bool updatePresence(PresenceTimer& t, const uint16_t minMm[3], const uint32_t frameAgeMs[3],
                    uint16_t detectMm, uint16_t holdMs, uint32_t nowMs);
