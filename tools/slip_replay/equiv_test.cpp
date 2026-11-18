// End-to-end equivalence check (CI): simulates the firmware control loop's order for the data
// path the slip detector uses (control.cpp: tofLogStep -> logStep -> slipUpdate every 10 ms, frame
// records per sensor update at any 1 ms tick), writes the recording with the firmware's CSV
// formatters, and prints the seqs where the "firmware" detector fired. The shell wrapper replays
// the files with slip_replay and requires the same seqs.
//   equiv_test <out_dir> [grasp]   -> writes rec.csv, rec_tof.csv, firmware_events.txt
// "grasp": a full grasp around the same hold (IDLE, START, PRE_TOUCH approach/slow, GRASP,
// MONITOR with operator marks, OPEN, IDLE), for the dashboard and analysis-pipeline chain test.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "fsm_core.h"
#include "log_core.h"
#include "slip_core.h"

namespace {
unsigned seed = 7;
float urand() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 65535.0f; }
int16_t milli(float v) { return (int16_t)lroundf(v * 1000.0f); }
int16_t tenth(float v) { return (int16_t)lroundf(v * 10.0f); }
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: equiv_test <out_dir>\n"); return 2; }
  const std::string dir = argv[1];
  const bool grasp = argc > 2 && !std::strcmp(argv[2], "grasp");
  const uint32_t T0 = grasp ? 3000 : 0;            // grasp mode: phases before the hold
  FILE* rows = std::fopen((dir + "/rec.csv").c_str(), "w");
  FILE* tofs = std::fopen((dir + "/rec_tof.csv").c_str(), "w");
  FILE* ev = std::fopen((dir + "/firmware_events.txt").c_str(), "w");
  if (!rows || !tofs || !ev) return 2;
  std::fprintf(rows, "# equivalence test\n%s\n", LOG_CSV_HEADER);
  std::fprintf(tofs, "# equivalence test\n%s\n", TOF_CSV_HEADER);

  SlipParams p = slipDefaults();
  SlipState st; slipReset(st);
  uint32_t rowNext = 0, tofNext = 0;
  uint16_t frame[3][64]; uint8_t fvalid = 7;
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) frame[s][z] = (uint16_t)(150 + z);
  uint16_t handFrame[3][64]; bool handNew = false, handHave = false; uint8_t handValid = 0;
  float force[6] = {1.8f, 1.2f, 0.9f, 0.0f, 0.6f, 0.0f};
  char line[2048];
  // Scenario (100 Hz rows): 0-2 s GRASP, then MONITOR with: noise; 6 s pull (forces drop 60 %
  // within 150 ms); 10 s object moves in sensor 2 view; 14 s glitch frame on sensor 1; 18 s twist
  // (sensor 3 zones change + small force wiggle); 22 s end.
  const uint32_t TEND = grasp ? 22000 + T0 + 3000 : 22000;
  for (uint32_t tt = 0; tt < TEND; tt++) {
    const uint32_t t = tt < T0 ? 0 : tt - T0;      // scenario time of the hold part
    // A sensor update (one frame record per sensor, sensors in turn, ~20 Hz each).
    if (tt % 17 == 0) {
      const int s = (int)((tt / 17) % 3);
      uint16_t (&fr)[64] = frame[s];
      for (int z = 0; z < 64; z++) fr[z] = (uint16_t)(150 + z + (int)(urand() * 6.0f) - 3);
      if (t >= 10000 && t < 13000 && s == 1) for (int z = 8; z < 20; z++) fr[z] = (uint16_t)(fr[z] + 60);
      if (t >= 14000 && t < 14017 && s == 0) for (int z = 0; z < 10; z++) fr[z] = 0;
      if (t >= 18000 && t < 20000 && s == 2) for (int z = 20; z < 40; z++) fr[z] = (uint16_t)(fr[z] + 30 + (t % 7));
      TofFrameRec f{}; f.seq = tofNext++; f.tMs = tt; f.rowSeq = rowNext; f.pos = 9000; f.valid = fvalid;
      std::memcpy(f.mm, frame, sizeof f.mm);
      tofCsvLine(f, line, sizeof line); std::fputs(line, tofs);
      std::memcpy(handFrame, frame, sizeof handFrame); handValid = fvalid; handNew = true; handHave = true;
    }
    if (tt % 10) continue;
    // logStep: the row (forces rounded as the firmware logs them).
    State state = t >= 2000 ? State::MONITOR : State::GRASP;
    uint8_t step = 0;
    if (grasp) {
      if (tt < 1000) state = State::IDLE;
      else if (tt < 1300) state = State::START;
      else if (tt < 2300) { state = State::PRE_TOUCH; step = 1; }
      else if (tt < 2800) { state = State::PRE_TOUCH; step = 2; }
      else if (tt < T0 + 2000) state = State::GRASP;
      else if (tt < T0 + 22000) state = State::MONITOR;
      else if (tt < T0 + 23000) state = State::OPEN;
      else state = State::IDLE;
    }
    float k = 1.0f;
    if (t >= 6000 && t < 6150) k = 1.0f - 0.6f * (float)(t - 6000) / 150.0f;
    else if (t >= 6150 && t < 8000) k = 0.4f;
    else if (t >= 18000 && t < 18300) k = 1.0f - 0.2f * std::sin((float)(t - 18000) / 30.0f);
    LogRow r{}; r.cls = 1; r.seq = rowNext++; r.tMs = tt; r.state = (uint8_t)state; r.step = step; r.armed = 1;
    static State prevState = State::FAULT;
    if (grasp) {   // events as the firmware logs them: state changes, contact, grasp reached, marks
      if (state != prevState) { r.ev = LogEvent::STATE; r.evArg = (uint8_t)state; }
      else if (tt == T0 + 1000) { r.ev = LogEvent::CONTACT; r.evArg = 1; r.evArg2 = 2; }
      else if (tt == T0 + 2010) r.ev = LogEvent::GRASP_REACHED;
      else if (tt == T0 + 6100) { r.ev = LogEvent::MARK; r.evArg = MARK_PULL; }
      else if (tt == T0 + 8000) { r.ev = LogEvent::MARK; r.evArg = MARK_END; }
      else if (tt == T0 + 10050) { r.ev = LogEvent::MARK; r.evArg = MARK_OTHER; }
      else if (tt == T0 + 13000) { r.ev = LogEvent::MARK; r.evArg = MARK_END; }
      else if (tt == T0 + 18100) { r.ev = LogEvent::MARK; r.evArg = MARK_TWIST; }
      else if (tt == T0 + 20000) { r.ev = LogEvent::MARK; r.evArg = MARK_END; }
      prevState = state;
    }
    for (int i = 0; i < 6; i++) {
      float f = force[i] * k * (1.0f - 0.004f * (float)t / 1000.0f) + (urand() - 0.5f) * 0.02f;
      if (grasp && (tt < T0 + 1000 || tt >= T0 + 22000)) f = 0.0f;   // no contact before / after
      r.fmN[i] = i == 5 ? INT16_MIN : milli(f); r.rmN[i] = milli(f);
    }
    r.objValid = 1; r.width10 = tenth(48.3f); r.range10 = tenth(120.0f + urand()); r.height10 = tenth(40.0f);
    for (int s = 0; s < 3; s++) r.rad10[s] = tenth(24.0f + urand());
    logCsvLine(r, line, sizeof line); std::fputs(line, rows);
    // slipUpdate: the logged row + the frame handed over since the previous row.
    SlipInput in{}; slipInputFromRow(r, in);
    in.frameNew = handNew; handNew = false;
    in.frame = handHave ? handFrame : nullptr; in.frameValid = handValid;
    if (slipStep(st, p, in).slip) std::fprintf(ev, "%lu\n", (unsigned long)r.seq);
  }
  std::fclose(rows); std::fclose(tofs); std::fclose(ev);
  return 0;
}
