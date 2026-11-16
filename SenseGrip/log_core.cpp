#include "log_core.h"
#include <climits>
#include <cstdarg>
#include <cstdio>
#include "contact_core.h"
#include "fsm_core.h"
#include "object_core.h"

const char LOG_CSV_HEADER[] =
    "seq,t_ms,state,step,armed,pos,speed,current_mA,load_pct,"
    "f1,f2,f3,f4,f5,f6,r1,r2,r3,r4,r5,r6,c1,c2,c3,"
    "tof_seq,obj_valid,width_mm,range_mm,height_mm,rad1,rad2,rad3,cls,shape,event";

void logInit(LogRing& r, LogRow* storage, uint32_t cap) { r.rows = storage; r.cap = cap; r.next = 0; }

void logPush(LogRing& r, LogRow row) {
  const uint32_t s = r.next;
  row.seq = s;
  r.rows[s % r.cap] = row;
  r.next = s + 1;                              // published after the row is complete
}

uint32_t logRead(const LogRing& r, uint32_t fromSeq, uint32_t max, LogRow* out, uint32_t& oldest, uint32_t& nextOut) {
  const uint32_t head = r.next;
  oldest = head > r.cap ? head - r.cap : 0;
  uint32_t s = fromSeq < oldest ? oldest : fromSeq;
  if (s > head) s = head;
  uint32_t n = 0;
  while (s < head && n < max) {
    out[n] = r.rows[s % r.cap];
    if (out[n].seq != s) break;                // overwritten while copying
    // Still inside the window after the copy? (the writer may have lapped us)
    const uint32_t h2 = r.next;
    if (h2 > r.cap && s < h2 - r.cap) break;
    n++; s++;
  }
  nextOut = s;
  return n;
}

namespace {
struct W {
  char* b; int len; int n = 0;
  void put(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};
void W::put(const char* fmt, ...) {
  if (n >= len) return;
  va_list ap; va_start(ap, fmt);
  const int k = std::vsnprintf(b + n, (size_t)(len - n), fmt, ap);
  va_end(ap);
  if (k > 0) n = (n + k < len) ? n + k : len - 1;
}
void milli(W& w, int16_t v) {                  // ",<v/1000 with 3 decimals>" or ","
  if (v == INT16_MIN) { w.put(","); return; }
  const int a = v < 0 ? -v : v;
  w.put(",%s%d.%03d", v < 0 ? "-" : "", a / 1000, a % 1000);
}
void tenth(W& w, int16_t v) {
  if (v == INT16_MIN) { w.put(","); return; }
  const int a = v < 0 ? -v : v;
  w.put(",%s%d.%d", v < 0 ? "-" : "", a / 10, a % 10);
}
const char* stepName(uint8_t s) { return s == 1 ? "approach" : s == 2 ? "slow" : "-"; }
const char* fcName(uint8_t f) { return f == 1 ? "proximal" : f == 2 ? "distal" : f == 3 ? "both" : "none"; }
}  // namespace

int logCsvLine(const LogRow& r, char* buf, int len) {
  W w{buf, len};
  w.put("%lu,%lu,%s,%s,%u,%ld,%d,%d", (unsigned long)r.seq, (unsigned long)r.tMs, stateName((State)r.state),
        stepName(r.step), (unsigned)r.armed, (long)r.pos, (int)r.speed, (int)r.currentMa);
  tenth(w, r.loadPermille);
  for (int i = 0; i < 6; i++) milli(w, r.fmN[i]);
  for (int i = 0; i < 6; i++) milli(w, r.rmN[i]);
  for (int f = 0; f < 3; f++) w.put(",%u", (unsigned)((r.contacts >> f) & 1));
  w.put(",%u,%u", (unsigned)r.tofSeq, (unsigned)r.objValid);
  tenth(w, r.width10); tenth(w, r.range10); tenth(w, r.height10);
  for (int s = 0; s < 3; s++) tenth(w, r.rad10[s]);
  w.put(",%s,%s,", r.cls <= 3 ? className((ObjClass)r.cls) : "", r.objValid ? shapeName((Shape)r.shape) : "");
  switch (r.ev) {
    case LogEvent::STATE: w.put("state:%s", stateName((State)r.evArg)); break;
    case LogEvent::CONTACT: w.put("contact:F%u:%s", (unsigned)r.evArg, fcName(r.evArg2)); break;
    case LogEvent::GRASP_REACHED: w.put("grasp_reached"); break;
    default: break;
  }
  w.put("\n");
  return w.n;
}
