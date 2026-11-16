// Grasp data log (phase 4a): one row per 100 Hz control step in a ring, served as CSV.
// Single writer (control task), single reader (web task), lock-free: each row carries its seq
// and the reader drops rows that were overwritten while it copied them. Pure C++.
#pragma once
#include <stdint.h>

enum class LogEvent : uint8_t { NONE, STATE, CONTACT, GRASP_REACHED };

struct LogRow {
  uint32_t seq, tMs;
  uint8_t state, step, armed;                 // State; PreTouchStep + 1 (0 = not in PRE_TOUCH); 0/1
  int32_t pos; int16_t speed, currentMa, loadPermille;
  int16_t fmN[6], rmN[6];                     // filtered / raw force, printed order FSR1..6, mN; INT16_MIN = empty
  uint8_t contacts;                           // bit f: finger f+1 in contact (debounced)
  uint16_t tofSeq; uint8_t objValid;
  int16_t width10, range10, height10, rad10[3];   // 0.1 mm; INT16_MIN = empty
  uint8_t cls, shape;                         // ObjClass 0..3 (255 = none), Shape
  LogEvent ev; uint8_t evArg, evArg2;         // STATE: new State; CONTACT: finger 1..3, FirstContact
};

constexpr uint32_t LOG_CAP = 30000;           // ~5 min at 100 Hz

struct LogRing { LogRow* rows; uint32_t cap; volatile uint32_t next; };   // next = seq of the next push

void logInit(LogRing& r, LogRow* storage, uint32_t cap);
void logPush(LogRing& r, LogRow row);         // row.seq = r.next; store; then ++next
// Copies up to max rows from fromSeq (clamped to the oldest still held). Returns the count; sets
// oldest and nextOut (seq after the last copied row). A row overwritten during the copy ends it.
uint32_t logRead(const LogRing& r, uint32_t fromSeq, uint32_t max, LogRow* out, uint32_t& oldest, uint32_t& nextOut);

extern const char LOG_CSV_HEADER[];           // 35 column names, no newline
int logCsvLine(const LogRow& row, char* buf, int len);   // one CSV line incl. '\n'; returns length
