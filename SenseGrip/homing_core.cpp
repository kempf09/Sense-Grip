#include "homing_core.h"

namespace {
int32_t floorDiv(int32_t a, int32_t b) { int32_t q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
int32_t roundDiv(int32_t a, int32_t b) { return floorDiv(a + b / 2, b); }
}  // namespace

HomeResult homeResolve(int32_t rawAtTouch, int32_t touchRef, int32_t maxResidual) {
  HomeResult r;
  r.posOffset = roundDiv(touchRef - rawAtTouch, COUNTS_PER_TURN) * COUNTS_PER_TURN;
  r.residual = rawAtTouch + r.posOffset - touchRef;
  r.ok = r.residual <= maxResidual && r.residual >= -maxResidual;
  r.why = r.ok ? nullptr : "touch point too far from the saved touch reference";
  return r;
}

int32_t turnOf(int32_t logicalPos, int32_t touchRef) { return floorDiv(logicalPos - touchRef, COUNTS_PER_TURN); }

bool stalled(int32_t pos, int32_t windowStartPos, uint32_t windowAgeMs) {
  const int32_t d = pos - windowStartPos;
  return windowAgeMs >= 300 && d < 10 && d > -10;
}

bool unstickAllowed(bool homed, int32_t pos, int32_t openZero, bool closePositive,
                    bool fingersVisible, bool anyFsrContact) {
  if (anyFsrContact) return false;
  if (homed) return closePositive ? pos <= openZero : pos >= openZero;
  return !fingersVisible;
}
