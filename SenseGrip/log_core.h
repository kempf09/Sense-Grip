// Grasp data log (phase 4a): one row per 100 Hz control step in a ring, served as CSV.
// Single writer (control task), single reader (web task), lock-free: each row carries its seq
// and the reader drops rows that were overwritten while it copied them. Pure C++.
#pragma once
#include <stdint.h>

// Event column. The text form is fixed (recordings depend on it):
//   STATE      state:<NAME>                 CONTACT  contact:F<n>:<proximal|distal|both>
//   GRASP_REACHED grasp_reached             MARK     mark:<pull|twist|slide|other|end> (operator)
//   SLIP       slip:<fsr|tof|fsr+tof> (arg bit0 fsr, bit1 tof)
//   TIGHTEN    tighten:<n>   TIGHTEN_DONE tighten_done   TIGHTEN_FAILED tighten_failed
enum class LogEvent : uint8_t { NONE, STATE, CONTACT, GRASP_REACHED, MARK, SLIP, TIGHTEN, TIGHTEN_DONE, TIGHTEN_FAILED };
enum : uint8_t { MARK_PULL = 1, MARK_TWIST, MARK_SLIDE, MARK_OTHER, MARK_END };

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

// Events waiting for a row: one event per row, in order (a mark is never lost behind a state change).
constexpr int LOG_EVENT_FIFO = 16;
struct LogEventFifo { LogEvent ev[LOG_EVENT_FIFO]; uint8_t a[LOG_EVENT_FIFO], a2[LOG_EVENT_FIFO]; uint8_t head, n; };
void logEventsReset(LogEventFifo& q);
bool logEventPush(LogEventFifo& q, LogEvent e, uint8_t a, uint8_t a2);   // false when full (event dropped)
bool logEventPop(LogEventFifo& q, LogEvent& e, uint8_t& a, uint8_t& a2);

// Raw SEN0628 frame stream: one record per new frame set (any sensor updated), recorded whether
// or not the gripper is homed. rowSeq = seq of the data-log row written next (links the streams).
struct TofFrameRec { uint32_t seq, tMs, rowSeq; int32_t pos; uint8_t valid; uint16_t mm[3][64]; };
constexpr uint32_t TOF_LOG_CAP = 15000;       // one record per sensor update (~30-60/s): ~4-8 min, 6 MB PSRAM
struct TofRing { TofFrameRec* rows; uint32_t cap; volatile uint32_t next; };
void tofLogInit(TofRing& r, TofFrameRec* storage, uint32_t cap);
void tofLogPush(TofRing& r, TofFrameRec f);
uint32_t tofLogRead(const TofRing& r, uint32_t fromSeq, uint32_t max, TofFrameRec* out, uint32_t& oldest, uint32_t& nextOut);
extern const char TOF_CSV_HEADER[];           // seq,t_ms,row_seq,pos,valid,s1_z00..s3_z63 (raw zone index)
int tofCsvLine(const TofFrameRec& f, char* buf, int len);
