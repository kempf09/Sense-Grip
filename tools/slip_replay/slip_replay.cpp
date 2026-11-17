// Replays a dashboard recording (<name>.csv + optional <name>_tof.csv) through the firmware's
// slip detector and prints per-row features and events as CSV.
//   slip_replay [--param name=value ...] [--force-active] --rows <name>.csv [--tof <name>_tof.csv] > out.csv
//   slip_replay --print-defaults
// Equivalence with the firmware by construction: rows are parsed back with logParseCsvLine and
// turned into the detector input with slipInputFromRow, exactly as the firmware does with the row
// it has just logged; frames are parsed with tofParseCsvLine. At main-file row seq S the detector
// gets every frame with row_seq <= S not used yet (the firmware hands a frame to the row written
// next, which is the row_seq it logs), the last one being the current frame.
// --force-active runs the detector on every row (manual recordings never reach MONITOR).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include "fsm_core.h"
#include "log_core.h"
#include "slip_core.h"

namespace {
int die(const char* m) { std::fprintf(stderr, "slip_replay: %s\n", m); return 2; }
std::string fmt(float v, const char* f) { if (!(v == v)) return ""; char b[32]; std::snprintf(b, sizeof b, f, (double)v); return b; }

// Reads data lines (skips '#' comments) and checks the header line.
bool readCsv(const char* path, const char* header, std::vector<std::string>& lines, std::string& err) {
  std::ifstream f(path);
  if (!f) { err = std::string("cannot read ") + path; return false; }
  std::string line; bool haveHeader = false;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    if (!haveHeader) {
      if (line != header) { err = std::string("unexpected header in ") + path + " (not a SenseGrip recording of this format)"; return false; }
      haveHeader = true; continue;
    }
    lines.push_back(line);
  }
  if (!haveHeader) { err = std::string("no header in ") + path; return false; }
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  SlipParams p = slipDefaults();
  const char* rowsPath = nullptr; const char* tofPath = nullptr; bool forceActive = false;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--print-defaults")) {
      for (int k = 0; slipParamName(k); k++) std::printf("%s=%g\n", slipParamName(k), (double)slipParamGet(p, slipParamName(k)));
      return 0;
    } else if (!std::strcmp(argv[i], "--param") && i + 1 < argc) {
      std::string a = argv[++i]; const size_t eq = a.find('=');
      if (eq == std::string::npos) return die("--param needs name=value");
      if (!slipParamSet(p, a.substr(0, eq).c_str(), std::strtof(a.c_str() + eq + 1, nullptr))) return die(("unknown parameter " + a).c_str());
    } else if (!std::strcmp(argv[i], "--rows") && i + 1 < argc) rowsPath = argv[++i];
    else if (!std::strcmp(argv[i], "--tof") && i + 1 < argc) tofPath = argv[++i];
    else if (!std::strcmp(argv[i], "--force-active")) forceActive = true;
    else return die("usage: slip_replay [--param name=value ...] [--force-active] --rows <name>.csv [--tof <name>_tof.csv] | --print-defaults");
  }
  if (!rowsPath) return die("--rows is required");
  std::vector<std::string> rowLines, tofLines; std::string err;
  if (!readCsv(rowsPath, LOG_CSV_HEADER, rowLines, err)) return die(err.c_str());
  if (tofPath && !readCsv(tofPath, TOF_CSV_HEADER, tofLines, err)) return die(err.c_str());

  std::vector<LogRow> rows; rows.reserve(rowLines.size());
  for (size_t i = 0; i < rowLines.size(); i++) {
    LogRow r;
    if (!logParseCsvLine(rowLines[i].c_str(), r)) { std::fprintf(stderr, "slip_replay: malformed row %zu in %s\n", i + 1, rowsPath); return 2; }
    rows.push_back(r);
  }
  std::vector<TofFrameRec> frames; frames.reserve(tofLines.size());
  for (size_t i = 0; i < tofLines.size(); i++) {
    TofFrameRec f;
    if (!tofParseCsvLine(tofLines[i].c_str(), f)) { std::fprintf(stderr, "slip_replay: malformed frame %zu in %s\n", i + 1, tofPath); return 2; }
    frames.push_back(f);
  }
  {   // warnings: the replay can only match the firmware on complete recordings
    int jumps = 0, monitor = 0;
    for (size_t i = 0; i < rows.size(); i++) {
      if (i && rows[i].seq != rows[i - 1].seq + 1) jumps++;
      if (rows[i].state == (uint8_t)State::MONITOR) monitor++;
    }
    if (jumps) std::fprintf(stderr, "warning: %d seq jump(s) in %s: detector history differs from the firmware there\n", jumps, rowsPath);
    if (!monitor && !forceActive) std::fprintf(stderr, "warning: no MONITOR rows in %s: the detector never runs (use --force-active for manual recordings)\n", rowsPath);
    if ((monitor || forceActive) && frames.empty()) std::fprintf(stderr, "warning: no frames for %s: the ToF feature is off in this replay\n", rowsPath);
  }

  SlipState st; slipReset(st);
  static uint16_t frame[3][64]; uint8_t fvalid = 0; bool haveFrame = false; size_t ti = 0;
  std::printf("seq,t_ms,state,active,fsr_drop,fsr_flag,tof_zones,tof_disp_mm,tof_flag,slip,reason\n");
  for (const LogRow& r : rows) {
    SlipInput in{};
    slipInputFromRow(r, in);
    if (forceActive) in.active = true;
    while (ti < frames.size() && frames[ti].rowSeq <= r.seq) {
      std::memcpy(frame, frames[ti].mm, sizeof frame); fvalid = frames[ti].valid;
      haveFrame = true; in.frameNew = true; ti++;
    }
    in.frame = haveFrame ? frame : nullptr; in.frameValid = fvalid;
    const SlipOut o = slipStep(st, p, in);
    const char* reason = !o.slip ? "" : o.reason == (SLIP_FSR | SLIP_TOF) ? "fsr+tof" : (o.reason & SLIP_FSR) ? "fsr" : "tof";
    std::printf("%lu,%lu,%s,%d,%s,%d,%d,%s,%d,%d,%s\n", (unsigned long)r.seq, (unsigned long)r.tMs, stateName((State)r.state), o.armed ? 1 : 0,
                fmt(o.fsrDrop, "%.3f").c_str(), o.fsrFlag ? 1 : 0, o.tofZones, fmt(o.tofDispMm, "%.1f").c_str(),
                o.tofFlag ? 1 : 0, o.slip ? 1 : 0, reason);
  }
  return 0;
}
