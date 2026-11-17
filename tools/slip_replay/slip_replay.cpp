// Replays a dashboard recording (<name>.csv + optional <name>_tof.csv) through the firmware's
// slip detector (SenseGrip/slip_core.cpp) and prints per-row features and events as CSV.
//   slip_replay [--param name=value ...] --rows <name>.csv [--tof <name>_tof.csv] > out.csv
//   slip_replay --print-defaults
// Frames: at main-file row seq S the detector sees every frame with row_seq <= S not yet used
// (the firmware records a frame with row_seq = the next row's seq).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "slip_core.h"

namespace {
std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> v; std::string cur;
  for (char c : s) { if (c == ',') { v.push_back(cur); cur.clear(); } else if (c != '\r') cur += c; }
  v.push_back(cur);
  return v;
}
float num(const std::string& s) { if (s.empty()) return NAN; char* e = nullptr; const float v = std::strtof(s.c_str(), &e); return (e && *e == 0) ? v : NAN; }
int die(const char* m) { std::fprintf(stderr, "slip_replay: %s\n", m); return 2; }

struct Table { std::map<std::string, int> col; std::vector<std::vector<std::string>> rows; };
bool load(const char* path, Table& t) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line; bool header = false;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto v = split(line);
    if (!header) { for (size_t i = 0; i < v.size(); i++) t.col[v[i]] = (int)i; header = true; continue; }
    t.rows.push_back(v);
  }
  return header;
}
int need(const Table& t, const char* name) { auto it = t.col.find(name); return it == t.col.end() ? -1 : it->second; }
const std::string& at(const std::vector<std::string>& r, int i) { static const std::string e; return (i >= 0 && i < (int)r.size()) ? r[i] : e; }
std::string fmt(float v, const char* f) { if (!(v == v)) return ""; char b[32]; std::snprintf(b, sizeof b, f, (double)v); return b; }
}  // namespace

int main(int argc, char** argv) {
  SlipParams p = slipDefaults();
  const char* rowsPath = nullptr; const char* tofPath = nullptr;
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
    else return die("usage: slip_replay [--param name=value ...] --rows <name>.csv [--tof <name>_tof.csv] | --print-defaults");
  }
  if (!rowsPath) return die("--rows is required");
  Table R, T;
  if (!load(rowsPath, R)) return die("cannot read --rows file");
  if (tofPath && !load(tofPath, T)) return die("cannot read --tof file");
  const int cSeq = need(R, "seq"), cT = need(R, "t_ms"), cSt = need(R, "state"), cOv = need(R, "obj_valid");
  const int cRange = need(R, "range_mm"), cH = need(R, "height_mm");
  if (cSeq < 0 || cT < 0 || cSt < 0) return die("rows file lacks seq/t_ms/state");
  int cF[6], cRad[3];
  for (int i = 0; i < 6; i++) cF[i] = need(R, ("f" + std::to_string(i + 1)).c_str());
  for (int k = 0; k < 3; k++) cRad[k] = need(R, ("rad" + std::to_string(k + 1)).c_str());
  const int tRow = need(T, "row_seq"), tValid = need(T, "valid");
  int tZ[3][64];
  for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) {
    char n[16]; std::snprintf(n, sizeof n, "s%d_z%02d", s + 1, z); tZ[s][z] = need(T, n);
  }

  // Warnings (stderr, lines start with "warning:"): the replay can only match the firmware on
  // complete recordings.
  {
    long prev = -1; int jumps = 0, monitor = 0;
    for (const auto& r : R.rows) {
      const long s = std::strtol(at(r, cSeq).c_str(), nullptr, 10);
      if (prev >= 0 && s != prev + 1) jumps++;
      prev = s;
      if (at(r, cSt) == "MONITOR") monitor++;
    }
    if (jumps) std::fprintf(stderr, "warning: %d seq jump(s) in %s: detector history differs from the firmware there\n", jumps, rowsPath);
    if (!monitor) std::fprintf(stderr, "warning: no MONITOR rows in %s: the detector never runs\n", rowsPath);
    if (monitor && T.rows.empty()) std::fprintf(stderr, "warning: no frames for %s: the ToF feature is off in this replay\n", rowsPath);
  }
  SlipState st; slipReset(st);
  static uint16_t frame[3][64]; uint8_t fvalid = 0; bool haveFrame = false; size_t ti = 0;
  std::printf("seq,t_ms,state,active,fsr_drop,fsr_flag,tof_zones,tof_disp_mm,tof_flag,slip,reason\n");
  for (const auto& r : R.rows) {
    const long seq = std::strtol(at(r, cSeq).c_str(), nullptr, 10);
    SlipInput in{};
    in.tMs = (uint32_t)std::strtoul(at(r, cT).c_str(), nullptr, 10);
    in.active = at(r, cSt) == "MONITOR";
    for (int i = 0; i < 6; i++) in.force[i] = num(at(r, cF[i]));
    while (ti < T.rows.size() && std::strtol(at(T.rows[ti], tRow).c_str(), nullptr, 10) <= seq) {
      const auto& fr = T.rows[ti++];
      fvalid = (uint8_t)std::strtoul(at(fr, tValid).c_str(), nullptr, 10);
      for (int s = 0; s < 3; s++) for (int z = 0; z < 64; z++) frame[s][z] = (uint16_t)std::strtoul(at(fr, tZ[s][z]).c_str(), nullptr, 10);
      haveFrame = true; in.frameNew = true;
    }
    in.frame = haveFrame ? frame : nullptr; in.frameValid = fvalid;
    in.objValid = at(r, cOv) == "1";
    in.rangeMm = num(at(r, cRange)); in.heightMm = num(at(r, cH));
    for (int k = 0; k < 3; k++) in.rad[k] = num(at(r, cRad[k]));
    const SlipOut o = slipStep(st, p, in);
    const char* reason = !o.slip ? "" : o.reason == (SLIP_FSR | SLIP_TOF) ? "fsr+tof" : (o.reason & SLIP_FSR) ? "fsr" : "tof";
    std::printf("%ld,%u,%s,%d,%s,%d,%d,%s,%d,%d,%s\n", seq, in.tMs, at(r, cSt).c_str(), o.armed ? 1 : 0,
                fmt(o.fsrDrop, "%.3f").c_str(), o.fsrFlag ? 1 : 0, o.tofZones, fmt(o.tofDispMm, "%.1f").c_str(),
                o.tofFlag ? 1 : 0, o.slip ? 1 : 0, reason);
  }
  return 0;
}
