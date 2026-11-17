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

namespace {
// Single-writer / single-reader ring shared by both streams (each record carries its seq).
template <class T, class R> void ringInit(R& r, T* st, uint32_t cap) { r.rows = st; r.cap = cap; r.next = 0; }
template <class T, class R> void ringPush(R& r, T rec) {
  const uint32_t s = r.next;
  rec.seq = s;
  r.rows[s % r.cap] = rec;
  r.next = s + 1;                              // published after the record is complete
}
template <class T, class R>
uint32_t ringRead(const R& r, uint32_t fromSeq, uint32_t max, T* out, uint32_t& oldest, uint32_t& nextOut) {
  const uint32_t head = r.next;
  oldest = head > r.cap ? head - r.cap : 0;
  uint32_t s = fromSeq < oldest ? oldest : fromSeq;
  if (s > head) s = head;
  uint32_t n = 0;
  while (s < head && n < max) {
    out[n] = r.rows[s % r.cap];
    if (out[n].seq != s) break;                // overwritten while copying
    const uint32_t h2 = r.next;                // still inside the window after the copy?
    if (h2 > r.cap && s < h2 - r.cap) break;
    n++; s++;
  }
  nextOut = s;
  return n;
}
}  // namespace

void logInit(LogRing& r, LogRow* storage, uint32_t cap) { ringInit(r, storage, cap); }
void logPush(LogRing& r, LogRow row) { ringPush(r, row); }
uint32_t logRead(const LogRing& r, uint32_t fromSeq, uint32_t max, LogRow* out, uint32_t& oldest, uint32_t& nextOut) {
  return ringRead(r, fromSeq, max, out, oldest, nextOut);
}
void tofLogInit(TofRing& r, TofFrameRec* storage, uint32_t cap) { ringInit(r, storage, cap); }
void tofLogPush(TofRing& r, TofFrameRec f) { ringPush(r, f); }
uint32_t tofLogRead(const TofRing& r, uint32_t fromSeq, uint32_t max, TofFrameRec* out, uint32_t& oldest, uint32_t& nextOut) {
  return ringRead(r, fromSeq, max, out, oldest, nextOut);
}

void logEventsReset(LogEventFifo& q) { q.head = 0; q.n = 0; }
bool logEventPush(LogEventFifo& q, LogEvent e, uint8_t a, uint8_t a2) {
  if (q.n >= LOG_EVENT_FIFO) return false;
  const int i = (q.head + q.n) % LOG_EVENT_FIFO;
  q.ev[i] = e; q.a[i] = a; q.a2[i] = a2; q.n++;
  return true;
}
bool logEventPop(LogEventFifo& q, LogEvent& e, uint8_t& a, uint8_t& a2) {
  if (!q.n) return false;
  e = q.ev[q.head]; a = q.a[q.head]; a2 = q.a2[q.head];
  q.head = (uint8_t)((q.head + 1) % LOG_EVENT_FIFO); q.n--;
  return true;
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
    case LogEvent::MARK: {
      static const char* const M[6] = {"?", "pull", "twist", "slide", "other", "end"};
      w.put("mark:%s", M[r.evArg <= 5 ? r.evArg : 0]); break;
    }
    case LogEvent::SLIP: w.put("slip:%s", (r.evArg & 3) == 3 ? "fsr+tof" : (r.evArg & 1) ? "fsr" : "tof"); break;
    case LogEvent::TIGHTEN: w.put("tighten:%u", (unsigned)r.evArg); break;
    case LogEvent::TIGHTEN_DONE: w.put("tighten_done"); break;
    case LogEvent::TIGHTEN_FAILED: w.put("tighten_failed"); break;
    default: break;
  }
  w.put("\n");
  return w.n;
}

const char TOF_CSV_HEADER[] = "seq,t_ms,row_seq,pos,valid,s1_z00,s1_z01,s1_z02,s1_z03,s1_z04,s1_z05,s1_z06,s1_z07,s1_z08,s1_z09,s1_z10,s1_z11,s1_z12,s1_z13,s1_z14,s1_z15,s1_z16,s1_z17,s1_z18,s1_z19,s1_z20,s1_z21,s1_z22,s1_z23,s1_z24,s1_z25,s1_z26,s1_z27,s1_z28,s1_z29,s1_z30,s1_z31,s1_z32,s1_z33,s1_z34,s1_z35,s1_z36,s1_z37,s1_z38,s1_z39,s1_z40,s1_z41,s1_z42,s1_z43,s1_z44,s1_z45,s1_z46,s1_z47,s1_z48,s1_z49,s1_z50,s1_z51,s1_z52,s1_z53,s1_z54,s1_z55,s1_z56,s1_z57,s1_z58,s1_z59,s1_z60,s1_z61,s1_z62,s1_z63,s2_z00,s2_z01,s2_z02,s2_z03,s2_z04,s2_z05,s2_z06,s2_z07,s2_z08,s2_z09,s2_z10,s2_z11,s2_z12,s2_z13,s2_z14,s2_z15,s2_z16,s2_z17,s2_z18,s2_z19,s2_z20,s2_z21,s2_z22,s2_z23,s2_z24,s2_z25,s2_z26,s2_z27,s2_z28,s2_z29,s2_z30,s2_z31,s2_z32,s2_z33,s2_z34,s2_z35,s2_z36,s2_z37,s2_z38,s2_z39,s2_z40,s2_z41,s2_z42,s2_z43,s2_z44,s2_z45,s2_z46,s2_z47,s2_z48,s2_z49,s2_z50,s2_z51,s2_z52,s2_z53,s2_z54,s2_z55,s2_z56,s2_z57,s2_z58,s2_z59,s2_z60,s2_z61,s2_z62,s2_z63,s3_z00,s3_z01,s3_z02,s3_z03,s3_z04,s3_z05,s3_z06,s3_z07,s3_z08,s3_z09,s3_z10,s3_z11,s3_z12,s3_z13,s3_z14,s3_z15,s3_z16,s3_z17,s3_z18,s3_z19,s3_z20,s3_z21,s3_z22,s3_z23,s3_z24,s3_z25,s3_z26,s3_z27,s3_z28,s3_z29,s3_z30,s3_z31,s3_z32,s3_z33,s3_z34,s3_z35,s3_z36,s3_z37,s3_z38,s3_z39,s3_z40,s3_z41,s3_z42,s3_z43,s3_z44,s3_z45,s3_z46,s3_z47,s3_z48,s3_z49,s3_z50,s3_z51,s3_z52,s3_z53,s3_z54,s3_z55,s3_z56,s3_z57,s3_z58,s3_z59,s3_z60,s3_z61,s3_z62,s3_z63";

int tofCsvLine(const TofFrameRec& f, char* buf, int len) {
  W w{buf, len};
  w.put("%lu,%lu,%lu,%ld,%u", (unsigned long)f.seq, (unsigned long)f.tMs, (unsigned long)f.rowSeq, (long)f.pos, (unsigned)f.valid);
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) w.put(",%u", (unsigned)f.mm[s][z]);
  w.put("\n");
  return w.n;
}
