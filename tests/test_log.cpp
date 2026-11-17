#include "check.h"
#include "log_core.h"
#include <climits>
#include <cstring>
#include <string>
#include <vector>

static int fields(const std::string& s) { int n = 1; for (char c : s) if (c == ',') n++; return n; }
static std::vector<std::string> split(std::string s) {
  if (!s.empty() && s.back() == '\n') s.pop_back();
  std::vector<std::string> v; std::string cur;
  for (char c : s) { if (c == ',') { v.push_back(cur); cur.clear(); } else cur += c; }
  v.push_back(cur); return v;
}
static LogRow blank() {
  LogRow r{}; r.cls = 255;
  for (int i = 0; i < 6; i++) { r.fmN[i] = 0; r.rmN[i] = 0; }
  return r;
}

TEST(log_push_read_basic) {
  static LogRow st[10]; LogRing r; logInit(r, st, 10);
  for (int i = 0; i < 5; i++) { LogRow x = blank(); x.tMs = 100 + i; logPush(r, x); }
  LogRow out[10]; uint32_t oldest = 99, nx = 99;
  CHECK_EQ(logRead(r, 0, 10, out, oldest, nx), 5u);
  CHECK_EQ(oldest, 0u); CHECK_EQ(nx, 5u);
  CHECK_EQ(out[0].seq, 0u); CHECK_EQ(out[4].seq, 4u); CHECK_EQ(out[4].tMs, 104u);
  CHECK_EQ(logRead(r, 5, 10, out, oldest, nx), 0u);
  CHECK_EQ(nx, 5u);
}
TEST(log_read_after_overwrite_reports_oldest) {
  static LogRow st[10]; LogRing r; logInit(r, st, 10);
  for (int i = 0; i < 25; i++) logPush(r, blank());
  LogRow out[20]; uint32_t oldest, nx;
  CHECK_EQ(logRead(r, 3, 20, out, oldest, nx), 10u);
  CHECK_EQ(oldest, 15u); CHECK_EQ(out[0].seq, 15u); CHECK_EQ(nx, 25u);
}
TEST(log_read_respects_max) {
  static LogRow st[10]; LogRing r; logInit(r, st, 10);
  for (int i = 0; i < 8; i++) logPush(r, blank());
  LogRow out[10]; uint32_t oldest, nx;
  CHECK_EQ(logRead(r, 2, 3, out, oldest, nx), 3u);
  CHECK_EQ(out[0].seq, 2u); CHECK_EQ(nx, 5u);
}
TEST(log_csv_header_columns) {
  const std::string h = LOG_CSV_HEADER;
  CHECK_EQ(fields(h), 35);
  CHECK(h.rfind("seq,t_ms,state,step,armed,pos,", 0) == 0);
  CHECK(h.size() > 15 && h.compare(h.size() - 15, 15, "cls,shape,event") == 0);
}
TEST(log_csv_line_values) {
  LogRow r = blank();
  r.seq = 7; r.tMs = 1234; r.state = 3; r.step = 0; r.armed = 1; r.pos = 5000;
  r.fmN[0] = 1234; r.rmN[5] = -20; r.contacts = 0x5;
  r.objValid = 1; r.width10 = 623; r.range10 = 1400; r.height10 = 450;
  r.rad10[0] = 311; r.rad10[1] = INT16_MIN; r.rad10[2] = 300;
  r.cls = 2; r.shape = 1; r.ev = LogEvent::STATE; r.evArg = 3;
  char b[512]; const int n = logCsvLine(r, b, sizeof b);
  CHECK(n > 0 && b[n - 1] == '\n');
  auto v = split(b);
  CHECK_EQ((int)v.size(), 35);
  CHECK(v[0] == "7"); CHECK(v[2] == "GRASP"); CHECK(v[3] == "-"); CHECK(v[4] == "1");
  CHECK(v[9] == "1.234");                                // f1
  CHECK(v[20] == "-0.020");                              // r6
  CHECK(v[21] == "1" && v[22] == "0" && v[23] == "1");   // c1..c3
  CHECK(v[26] == "62.3"); CHECK(v[29] == "31.1"); CHECK(v[30] == ""); CHECK(v[32] == "large");
  CHECK(v[33] == "round"); CHECK(v[34] == "state:GRASP");
}
TEST(log_csv_empty_fields) {
  LogRow r = blank();
  for (int i = 0; i < 6; i++) { r.fmN[i] = INT16_MIN; r.rmN[i] = INT16_MIN; }
  r.width10 = r.range10 = r.height10 = INT16_MIN;
  for (auto& x : r.rad10) x = INT16_MIN;
  r.ev = LogEvent::CONTACT; r.evArg = 2; r.evArg2 = 2;
  char b[512]; logCsvLine(r, b, sizeof b);
  auto v = split(b);
  CHECK_EQ((int)v.size(), 35);
  CHECK(v[9] == "" && v[20] == ""); CHECK(v[26] == ""); CHECK(v[32] == "");
  CHECK(v[34] == "contact:F2:distal");
  CHECK(std::string(b).find("nan") == std::string::npos);
}

// ---- phase 4: raw SEN0628 frame stream, marks, slip/tighten events ----
TEST(tof_log_push_read_and_csv) {
  static TofFrameRec st[4]; TofRing r; tofLogInit(r, st, 4);
  for (int i = 0; i < 6; i++) {
    TofFrameRec f{}; f.tMs = 10 * i; f.rowSeq = 100 + i; f.pos = -5; f.valid = 0x5;
    for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) f.mm[s][z] = (uint16_t)(s * 1000 + z);
    tofLogPush(r, f);
  }
  TofFrameRec out[8]; uint32_t oldest, nx;
  CHECK_EQ(tofLogRead(r, 0, 8, out, oldest, nx), 4u);
  CHECK_EQ(oldest, 2u); CHECK_EQ(out[0].seq, 2u); CHECK_EQ(nx, 6u);
  const std::string h = TOF_CSV_HEADER;
  CHECK_EQ(fields(h), 5 + 192);
  CHECK(h.rfind("seq,t_ms,row_seq,pos,valid,s1_z00,", 0) == 0);
  CHECK(h.compare(h.size() - 6, 6, "s3_z63") == 0);
  static char b[2048];
  const int n = tofCsvLine(out[0], b, sizeof b);
  CHECK(n > 0 && b[n - 1] == '\n');
  auto v = split(b);
  CHECK_EQ((int)v.size(), 197);
  CHECK(v[0] == "2" && v[2] == "102" && v[3] == "-5" && v[4] == "5");
  CHECK(v[5] == "0" && v[5 + 64] == "1000" && v[196] == "2063");
}
TEST(log_csv_marks_and_slip_events) {
  LogRow r = blank(); char b[512];
  r.ev = LogEvent::MARK; r.evArg = 2; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "mark:twist");
  r.evArg = 5; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "mark:end");
  r.ev = LogEvent::SLIP; r.evArg = 3; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "slip:fsr+tof");
  r.evArg = 1; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "slip:fsr");
  r.ev = LogEvent::TIGHTEN; r.evArg = 2; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "tighten:2");
  r.ev = LogEvent::TIGHTEN_DONE; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "tighten_done");
  r.ev = LogEvent::TIGHTEN_FAILED; logCsvLine(r, b, sizeof b); CHECK(split(b)[34] == "tighten_failed");
}
TEST(log_event_fifo_keeps_order_and_drops_when_full) {
  LogEventFifo q; logEventsReset(q);
  CHECK(logEventPush(q, LogEvent::MARK, 1, 0));
  CHECK(logEventPush(q, LogEvent::STATE, 3, 0));
  LogEvent e; uint8_t a, a2;
  CHECK(logEventPop(q, e, a, a2)); CHECK(e == LogEvent::MARK); CHECK_EQ((int)a, 1);
  CHECK(logEventPop(q, e, a, a2)); CHECK(e == LogEvent::STATE);
  CHECK(!logEventPop(q, e, a, a2));
  for (int i = 0; i < LOG_EVENT_FIFO; i++) CHECK(logEventPush(q, LogEvent::CONTACT, 1, 1));
  CHECK(!logEventPush(q, LogEvent::CONTACT, 1, 1));
}
