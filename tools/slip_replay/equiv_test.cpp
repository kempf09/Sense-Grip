// End-to-end equivalence check (CI): simulates the firmware control loop's order for the data
// path the slip detector uses (control.cpp: tofLogStep -> logStep -> slipUpdate every 10 ms, frame
// records per sensor update at any 1 ms tick), writes the recording with the firmware's CSV
// formatters, and prints the seqs where the "firmware" detector fired. The shell wrapper replays
// the files with slip_replay and requires the same seqs.
//   equiv_test <out_dir>   -> writes rec.csv, rec_tof.csv, firmware_events.txt
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
  for (uint32_t t = 0; t < 22000; t++) {
    // A sensor update (one frame record per sensor, sensors in turn, ~20 Hz each).
    if (t % 17 == 0) {
      const int s = (int)((t / 17) % 3);
      uint16_t (&fr)[64] = frame[s];
      for (int z = 0; z < 64; z++) fr[z] = (uint16_t)(150 + z + (int)(urand() * 6.0f) - 3);
      if (t >= 10000 && t < 13000 && s == 1) for (int z = 8; z < 20; z++) fr[z] = (uint16_t)(fr[z] + 60);
      if (t >= 14000 && t < 14017 && s == 0) for (int z = 0; z < 10; z++) fr[z] = 0;
      if (t >= 18000 && t < 20000 && s == 2) for (int z = 20; z < 40; z++) fr[z] = (uint16_t)(fr[z] + 30 + (t % 7));
      TofFrameRec f{}; f.seq = tofNext++; f.tMs = t; f.rowSeq = rowNext; f.pos = 9000; f.valid = fvalid;
      std::memcpy(f.mm, frame, sizeof f.mm);
      tofCsvLine(f, line, sizeof line); std::fputs(line, tofs);
      std::memcpy(handFrame, frame, sizeof handFrame); handValid = fvalid; handNew = true; handHave = true;
    }
    if (t % 10) continue;
    // logStep: the row (forces rounded as the firmware logs them).
    const bool monitor = t >= 2000;
    float k = 1.0f;
    if (t >= 6000 && t < 6150) k = 1.0f - 0.6f * (float)(t - 6000) / 150.0f;
    else if (t >= 6150 && t < 8000) k = 0.4f;
    else if (t >= 18000 && t < 18300) k = 1.0f - 0.2f * std::sin((float)(t - 18000) / 30.0f);
    LogRow r{}; r.cls = 1; r.seq = rowNext++; r.tMs = t; r.state = (uint8_t)(monitor ? State::MONITOR : State::GRASP);
    for (int i = 0; i < 6; i++) {
      const float f = force[i] * k * (1.0f - 0.004f * (float)t / 1000.0f) + (urand() - 0.5f) * 0.02f;
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
