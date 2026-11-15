// Power-up homing maths: the servo keeps its angle within a turn but loses the turn count.
// Close-to-touch gives a known point; the turn is the one that puts it nearest the saved touch
// reference. Pure C++.
#pragma once
#include <stdint.h>

constexpr int32_t COUNTS_PER_TURN = 4096;
struct HomeResult { bool ok; int32_t posOffset; int32_t residual; const char* why; };

// rawAtTouch: servo position where the fingertips touched; touchRef: saved logical touch position.
HomeResult homeResolve(int32_t rawAtTouch, int32_t touchRef, int32_t maxResidual);
// Turn index of a logical position relative to touchRef (floor division by 4096).
int32_t turnOf(int32_t logicalPos, int32_t touchRef);
// True when the position moved < 10 counts during a window of >= 300 ms while commanded.
bool stalled(int32_t pos, int32_t windowStartPos, uint32_t windowAgeMs);
// The maximum-current push is only for a jaw stuck past OPEN, never near closed and never with
// an FSR in contact: homed and at/beyond OPEN zero (opening side), or not homed and the
// SEN0628 cannot see the fingers (open region).
// A jaw stuck past OPEN does not move at all, so the push is only for a stall right at the start
// (msSinceStart <= UNSTICK_START_WINDOW_MS: one 300 ms stall window plus the first step).
constexpr uint32_t UNSTICK_START_WINDOW_MS = 800;
bool unstickAllowed(bool homed, int32_t pos, int32_t openZero, bool closePositive,
                    bool fingersVisible, bool anyFsrContact, uint32_t msSinceStart);
// The SEN0628 estimate is good to a few hundred counts: it conflicts with the homed frame only
// when the two positions differ by more than half a turn (a different turn index alone is not
// a conflict near a turn boundary).
bool estimateConflicts(int32_t estimatePos, int32_t homedPos);
