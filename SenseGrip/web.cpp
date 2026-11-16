// JSON API served from the web task. Never touches UART/ADC/I2C: motion, servo and
// config changes go through controlEnqueue(); sensor data comes from snapshots.
#include <Arduino.h>
#include <WebServer.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "config_core.h"
#include "control.h"
#include "fsm_core.h"
#include "fsr.h"
#include "json_util.h"
#include "servo.h"
#include "servo_proto.h"
#include "tof.h"
#include "sweep_store.h"
#include "web.h"
#include "web_pages.h"

static WebServer server(80);

// ---------------------------------------------------------------- JSON writer
// Bounded snprintf appender; `over` latches if the buffer filled.
struct Out {
  char* b; size_t cap; size_t n; bool over;
  Out(char* buf, size_t c) : b(buf), cap(c), n(0), over(false) { if (c) b[0] = '\0'; }
  void put(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (over) return;
    va_list ap; va_start(ap, fmt);
    int r = vsnprintf(b + n, cap - n, fmt, ap);
    va_end(ap);
    if (r < 0 || (size_t)r >= cap - n) over = true; else n += (size_t)r;
  }
  void flt(float v) { if (isnan(v) || isinf(v)) put("null"); else put("%.3f", (double)v); }
  void boolean(bool v) { put(v ? "true" : "false"); }
  void str(const char* s) {
    static char t[640];                 // 96-char text, all control chars -> 576 + NUL
    jsonEscape(s, t, sizeof(t));
    put("\"%s\"", t);
  }
};

static char statusBuf[8192];            // only the web task touches these (status incl. up to 21 blockers)
static char smallBuf[1536];

static void sendJson(int code, const char* body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void sendError(int code, const char* msg, const char* detail = nullptr) {
  char m[200];
  if (detail) snprintf(m, sizeof(m), "%s: %s", msg, detail); else snprintf(m, sizeof(m), "%s", msg);
  Out o(smallBuf, sizeof(smallBuf));
  o.put("{\"ok\":false,\"error\":"); o.str(m); o.put("}");
  sendJson(code, smallBuf);
}

static void sendOut(Out& o) {
  if (o.over) sendError(500, "status too large"); else sendJson(200, o.b);
}

static void sendQueued(const ControlRequest& r) {
  if (!controlEnqueue(r)) { sendError(503, "controller busy"); return; }
  sendJson(200, "{\"ok\":true,\"queued\":true}");
}

static ControlRequest blankReq(ControlRequest::Kind k) {
  ControlRequest r; memset(&r, 0, sizeof(r));
  r.kind = k; r.cmd = Cmd::NONE;
  return r;
}

static void fsmCmd(Cmd c) {
  ControlRequest r = blankReq(ControlRequest::FSM_CMD); r.cmd = c;
  sendQueued(r);
}

// ---------------------------------------------------------------- parsing
static bool parseI(const String& s, long long lo, long long hi, long long& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  errno = 0;
  long long v = strtoll(s.c_str(), &end, 10);
  if (errno || *end != '\0' || v < lo || v > hi) return false;
  out = v; return true;
}
static bool parseF(const String& s, float& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  float v = strtof(s.c_str(), &end);
  if (*end != '\0' || isnan(v) || isinf(v)) return false;
  out = v; return true;
}
static bool parseBool(const String& s, bool& out) {
  if (s == "1" || s == "true") { out = true; return true; }
  if (s == "0" || s == "false") { out = false; return true; }
  return false;
}
static bool parseMask(const String& s, uint64_t& out) {
  if (s.length() != 16) return false;
  uint64_t v = 0;
  for (size_t i = 0; i < 16; i++) {
    char c = s[i]; uint8_t d;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
    else return false;
    v = (v << 4) | d;
  }
  out = v; return true;
}

// ---------------------------------------------------------------- /api/status
static void handleStatus() {
  ControlStatus cs = controlGet();
  GripConfig cfg = controlGetConfig();
  ServoSnapshot sv = servoGet();
  FsrSnapshot fs = fsrGet();
  TofSnapshot tf = tofGet();
  const uint8_t bits = (uint8_t)(sv.fb.status | sv.replyStatus);   // same OR as the FSM input
  char st[48]; svStatusText(bits, st, sizeof(st));

  Out o(statusBuf, sizeof(statusBuf));
  o.put("{\"ok\":true,\"state\":"); o.str(stateName(cs.state));
  o.put(",\"faultFrom\":"); o.str(stateName(cs.faultFrom));
  o.put(",\"armed\":"); o.boolean(cs.armed);
  o.put(",\"openedSinceBoot\":"); o.boolean(cs.openedSinceBoot);
  o.put(",\"reason\":"); o.str(cs.reason);
  o.put(",\"faultReason\":"); o.str(cs.faultReason);
  o.put(",\"blockers\":[");
  for (uint8_t i = 0; i < cs.nBlockers; i++) { if (i) o.put(","); o.str(cs.blockers[i].text); }
  o.put("],\"lastAction\":{\"msg\":"); o.str(cs.lastActionMsg);
  o.put(",\"ok\":"); o.boolean(cs.lastActionOk);

  o.put("},\"servo\":{\"online\":"); o.boolean(sv.online);
  o.put(",\"ageMs\":%lu,\"position\":%ld,\"speed\":%d,\"load\":",
        (unsigned long)sv.ageMs, (long)sv.fb.position, (int)sv.fb.speed);
  o.flt(sv.fb.load);
  o.put(",\"current\":%d,\"voltage\":", (int)sv.fb.current); o.flt(sv.fb.voltage);
  o.put(",\"temp\":%u,\"status\":%u,\"replyStatus\":%u,\"statusText\":",
        (unsigned)sv.fb.temp, (unsigned)sv.fb.status, (unsigned)sv.replyStatus);
  o.str(st);
  if (sv.modeKnown) o.put(",\"mode\":%u", (unsigned)sv.mode); else o.put(",\"mode\":null");
  o.put(",\"fullAngle\":"); o.boolean(sv.fullAngle);
  o.put(",\"lastError\":"); o.str(sv.lastError);

  o.put("},\"fsr\":{\"force\":[");
  for (int i = 0; i < 6; i++) { if (i) o.put(","); o.flt(fs.force[i]); }
  o.put("],\"forceRaw\":[");
  for (int i = 0; i < 6; i++) { if (i) o.put(","); o.flt(fs.forceRaw[i]); }
  o.put("],\"filterDelayMs\":"); o.flt(filterGroupDelayMs(filterParamsOf(cfg), 10.0f));
  o.put(",\"limitForce\":"); o.flt(fs.limitForce);
  o.put(",\"raw\":[");
  for (int i = 0; i < 6; i++) o.put(i ? ",%u" : "%u", (unsigned)fs.raw[i]);
  o.put("],\"mv\":[");
  for (int i = 0; i < 6; i++) { if (i) o.put(","); o.flt(fs.mv[i]); }
  o.put("],\"ohms\":[");
  for (int i = 0; i < 6; i++) { if (i) o.put(","); o.flt(fs.ohms[i]); }
  o.put("],\"calibrated\":[");
  for (int i = 0; i < 6; i++) { if (i) o.put(","); o.boolean(fs.calibrated[i]); }
  o.put("],\"fingerForce\":[");
  for (int i = 0; i < 3; i++) { if (i) o.put(","); o.flt(fs.agg.fingerForce[i]); }
  o.put("],\"fingerContact\":[");
  for (int i = 0; i < 3; i++) { if (i) o.put(","); o.boolean(fs.contacts.stable[i]); }
  o.put("],\"avgContactForce\":"); o.flt(fs.agg.avgContactForce);
  o.put(",\"channelsInContact\":%u,\"maxForce\":", (unsigned)fs.agg.channelsInContact);
  o.flt(fs.agg.maxForce);

  o.put("},\"tof\":{\"online\":%u,\"objectPresent\":", (unsigned)tf.online);
  o.boolean(tf.objectPresent);
  o.put(",\"minMm\":[");
  for (int i = 0; i < 3; i++) {
    if (i) o.put(",");
    if (tf.s[i].online) o.put("%u", (unsigned)tf.s[i].minMm); else o.put("null");
  }
  o.put("]},\"limits\":{\"openZero\":%ld,\"closeLimit\":%ld,\"valid\":",
        (long)cfg.openZero, (long)cfg.closeLimit);
  o.boolean(cfg.limitsValid);
  o.put("},\"home\":{\"homed\":"); o.boolean(cs.homed);
  o.put(",\"homing\":"); o.boolean(cs.homing);
  o.put(",\"residual\":%ld,\"msg\":", (long)cs.homeResidual); o.str(cs.homeMsg);
  o.put(",\"sweeping\":"); o.boolean(cs.sweeping);
  o.put(",\"sweepN\":%ld,\"sweepLoaded\":", (long)cs.sweepN); o.boolean(cs.sweepLoaded);
  o.put(",\"estimate\":");
  if (cs.estValid) o.put("{\"pos\":%ld,\"turn\":%ld}", (long)cs.estPos, (long)cs.estTurn); else o.put("null");
  o.put("},\"diag\":{\"loopMaxUs\":%lu,\"posJumps\":%lu,\"servoBusErrors\":%lu,\"fsrSampleUsMax\":%lu}}",
        (unsigned long)cs.loopMaxUs, (unsigned long)cs.posJumps, (unsigned long)sv.busErrors,
        (unsigned long)cs.fsrSampleUsMax);
  sendOut(o);
}

// ---------------------------------------------------------------- /api/tof (chunked)
static void handleTof() {
  TofSnapshot tf = tofGet();
  uint32_t now = millis();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"sensors\":[");
  for (int i = 0; i < 3; i++) {
    const TofSensor& s = tf.s[i];
    Out o(smallBuf, sizeof(smallBuf));
    if (i) o.put(",");
    o.put("{\"online\":"); o.boolean(s.online);
    o.put(",\"initialized\":"); o.boolean(s.initialized);
    o.put(",\"initFailStage\":%u,\"initFails\":%lu,\"lastRc\":%u,\"readUsMax\":%lu,\"dropouts\":%lu,\"dropoutsMoving\":%lu",
          (unsigned)s.initFailStage, (unsigned long)s.initFails, (unsigned)s.lastRc,
          (unsigned long)s.readUsMax, (unsigned long)s.dropouts, (unsigned long)s.dropoutsMoving);
    o.put(",\"ok\":%lu,\"err\":%lu,\"ageMs\":", (unsigned long)s.ok, (unsigned long)s.err);
    if (s.online) o.put("%lu", (unsigned long)(now - s.lastGoodMs)); else o.put("null");
    o.put(",\"minMm\":");
    if (s.online) o.put("%u", (unsigned)s.minMm); else o.put("null");
    o.put(",\"mm\":[");
    for (int z = 0; z < 64; z++) {
      if (z) o.put(",");
      if (s.online) o.put("%u", (unsigned)s.mm[z]); else o.put("null");
    }
    o.put("]}");
    if (!o.over) server.sendContent(smallBuf);
  }
  server.sendContent("]}");
  server.sendContent("");               // terminate chunked body
}

// ---------------------------------------------------------------- config
enum FieldType : uint8_t { T_I32, T_U8, T_U16, T_U32, T_F, T_BOOL, T_MASK };
struct Field { const char* name; FieldType t; size_t off; uint8_t idx; };
#define F_(n, t) { #n, t, offsetof(GripConfig, n), 0 }
static const Field FIELDS[] = {
  F_(closePositive, T_BOOL),
  F_(approachSpeed, T_U16), F_(graspSpeed, T_U16), F_(openSpeed, T_U16),
  F_(closeTorqueLimit, T_U16), F_(openTorqueLimit, T_U16),
  F_(openToleranceCounts, T_U16), F_(limitSlackCounts, T_U16),
  F_(contactForceN, T_F), F_(contactSamples, T_U8),
  F_(graspForceN, T_F), F_(graspSamples, T_U8),
  F_(minActiveFsr, T_U8), F_(maxForceN, T_F),
  F_(detectMm, T_U16), F_(detectHoldMs, T_U16),
  F_(preTouchTimeoutMs, T_U32), F_(graspTimeoutMs, T_U32), F_(openTimeoutMs, T_U32),
  F_(fsrEnabled, T_U8),
  F_(fsrFilter, T_U8), F_(fsrFilterWindow, T_U8), F_(fsrFilterAlpha, T_F), F_(fsrKalmanQ, T_F), F_(fsrKalmanR, T_F),
  { "tofMask0", T_MASK, offsetof(GripConfig, tofMask), 0 },
  { "tofMask1", T_MASK, offsetof(GripConfig, tofMask), 1 },
  { "tofMask2", T_MASK, offsetof(GripConfig, tofMask), 2 },
};
#undef F_
static const size_t NFIELDS = sizeof(FIELDS) / sizeof(FIELDS[0]);

static bool isReadOnlyLimitField(const String& n) {
  return n == "openZero" || n == "closeLimit" || n == "limitsValid";
}

// Parses `v` into the field; false on bad value.
static bool assignField(GripConfig& c, const Field& f, const String& v) {
  uint8_t* base = (uint8_t*)&c + f.off;
  long long i; float x; bool b; uint64_t m;
  switch (f.t) {
    case T_I32:  if (!parseI(v, INT32_MIN, INT32_MAX, i)) return false; *(int32_t*)base = (int32_t)i; return true;
    case T_U8:   if (!parseI(v, 0, 255, i)) return false;               *(uint8_t*)base = (uint8_t)i; return true;
    case T_U16:  if (!parseI(v, 0, 65535, i)) return false;             *(uint16_t*)base = (uint16_t)i; return true;
    case T_U32:  if (!parseI(v, 0, 4294967295LL, i)) return false;      *(uint32_t*)base = (uint32_t)i; return true;
    case T_F:    if (!parseF(v, x)) return false;                       *(float*)base = x; return true;
    case T_BOOL: if (!parseBool(v, b)) return false;                    *(bool*)base = b; return true;
    case T_MASK: if (!parseMask(v, m)) return false;                    ((uint64_t*)base)[f.idx] = m; return true;
  }
  return false;
}

static void emitField(Out& o, const GripConfig& c, const Field& f) {
  const uint8_t* base = (const uint8_t*)&c + f.off;
  o.put(",\"%s\":", f.name);
  switch (f.t) {
    case T_I32:  o.put("%ld", (long)*(const int32_t*)base); break;
    case T_U8:   o.put("%u", (unsigned)*(const uint8_t*)base); break;
    case T_U16:  o.put("%u", (unsigned)*(const uint16_t*)base); break;
    case T_U32:  o.put("%lu", (unsigned long)*(const uint32_t*)base); break;
    case T_F:    o.flt(*(const float*)base); break;
    case T_BOOL: o.boolean(*(const bool*)base); break;
    case T_MASK: o.put("\"%016llx\"", (unsigned long long)((const uint64_t*)base)[f.idx]); break;
  }
}

static void handleConfigGet() {
  GripConfig c = controlGetConfig();
  Out o(smallBuf, sizeof(smallBuf));
  o.put("{\"ok\":true,\"openZero\":%ld,\"closeLimit\":%ld,\"limitsValid\":",
        (long)c.openZero, (long)c.closeLimit);
  o.boolean(c.limitsValid);
  for (size_t i = 0; i < NFIELDS; i++) emitField(o, c, FIELDS[i]);
  o.put("}");
  sendOut(o);
}

static void handleConfigPost() {
  GripConfig c = controlGetConfig();
  for (int a = 0; a < server.args(); a++) {
    String name = server.argName(a);
    if (isReadOnlyLimitField(name)) { sendError(400, "use setOpen/setClose", name.c_str()); return; }
    const Field* f = nullptr;
    for (size_t i = 0; i < NFIELDS; i++) if (name == FIELDS[i].name) { f = &FIELDS[i]; break; }
    if (!f) { sendError(400, "unknown field", name.c_str()); return; }
    if (!assignField(c, *f, server.arg(a))) { sendError(400, "invalid value for field", name.c_str()); return; }
  }
  clampConfig(c);
  ControlRequest r = blankReq(ControlRequest::CONFIG);
  r.cfg = c;
  sendQueued(r);
}

// ---------------------------------------------------------------- FSR calibration
static bool calAllowed() {
  State s = controlGet().state;
  if (s == State::IDLE || s == State::FAULT) return true;
  sendError(409, "calibration only in IDLE or FAULT");
  return false;
}

static bool parseChannel(int& id) {
  long long v;
  if (!server.hasArg("id") || !parseI(server.arg("id"), 0, 5, v)) { sendError(400, "invalid value for field", "id"); return false; }
  id = (int)v; return true;
}

static void handleCalGet() {
  Out o(smallBuf, sizeof(smallBuf));
  o.put("{\"ok\":true,\"cal\":[");
  for (int i = 0; i < 6; i++) {
    FsrCal c = fsrGetCal((uint8_t)i);
    if (i) o.put(",");
    o.put("{\"id\":%d,\"r\":", i); o.flt(c.fixedOhms);
    o.put(",\"f1\":"); o.flt(c.f1); o.put(",\"o1\":"); o.flt(c.o1);
    o.put(",\"f2\":"); o.flt(c.f2); o.put(",\"o2\":"); o.flt(c.o2);
    o.put(",\"z\":"); o.flt(c.zeroMv);
    o.put(",\"valid\":"); o.boolean(fsrCalValid(c)); o.put("}");
  }
  o.put("]}");
  sendOut(o);
}

static void handleCalPost() {
  int id; if (!parseChannel(id)) return;
  FsrCal c = fsrGetCal((uint8_t)id);
  struct { const char* n; float* p; } fl[] = {
    {"r", &c.fixedOhms}, {"f1", &c.f1}, {"o1", &c.o1}, {"f2", &c.f2}, {"o2", &c.o2}, {"z", &c.zeroMv}};
  for (int a = 0; a < server.args(); a++) {
    String name = server.argName(a);
    if (name == "id") continue;
    bool found = false;
    for (auto& f : fl) {
      if (name != f.n) continue;
      found = true;
      if (!parseF(server.arg(a), *f.p)) { sendError(400, "invalid value for field", name.c_str()); return; }
    }
    if (!found) { sendError(400, "unknown field", name.c_str()); return; }
  }
  if (!fsrCalValid(c)) { sendError(400, "calibration values out of range"); return; }
  if (!calAllowed()) return;
  if (!fsrSaveCal((uint8_t)id, c)) { sendError(500, "save failed"); return; }
  sendJson(200, "{\"ok\":true}");
}

static void handleZeroPost() {
  int id; if (!parseChannel(id)) return;
  if (!calAllowed()) return;
  if (!fsrZero((uint8_t)id)) { sendError(500, "zero failed"); return; }
  sendJson(200, "{\"ok\":true}");
}

// ---------------------------------------------------------------- sweep
// GET /api/sweep: positions, fingertip opening and finger zones of the stored sweep (chunked).
static void handleSweepGet() {
  const SweepModel& m = sweepModel();
  const SweepSample* s = sweepSamples();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  char b[96];
  const int n = (sweepLoaded() && s) ? m.n : 0;
  snprintf(b, sizeof b, "{\"ok\":true,\"n\":%d,\"openZero\":%ld,\"touchRef\":%ld,\"firstSeen\":%d,\"positions\":[",
           n, (long)m.openZero, (long)m.touchRef, n ? m.firstSeen : -1);
  server.sendContent(b);
  for (int i = 0; i < n; i++) { snprintf(b, sizeof b, i ? ",%ld" : "%ld", (long)s[i].pos); server.sendContent(b); }
  server.sendContent("],\"opening\":[");
  for (int i = 0; i < n; i++) { snprintf(b, sizeof b, i ? ",%.1f" : "%.1f", (double)m.opening[i]); server.sendContent(b); }
  server.sendContent("],\"fingerZones\":[");
  for (int k = 0; k < 3; k++) {
    snprintf(b, sizeof b, "%s\"%016llx\"", k ? "," : "", (unsigned long long)(n ? m.fingerZones[k] : 0));
    server.sendContent(b);
  }
  server.sendContent("]}");
  server.sendContent("");
}

// ---------------------------------------------------------------- servo
static void handleJog() {
  long long v;
  if (!server.hasArg("speed") || !parseI(server.arg("speed"), -(long long)SPEED_MAX, SPEED_MAX, v) || v == 0) {
    sendError(400, "invalid value for field", "speed"); return;
  }
  bool ignore = false;
  if (server.hasArg("ignoreLimits") && !parseBool(server.arg("ignoreLimits"), ignore)) {
    sendError(400, "invalid value for field", "ignoreLimits"); return;
  }
  ControlRequest r = blankReq(ControlRequest::JOG); r.speed = (int16_t)v; r.ignoreLimits = ignore;
  sendQueued(r);
}

static void simple(ControlRequest::Kind k) { sendQueued(blankReq(k)); }

static void handleEstop() {
  controlRequestEstop();                // flag path first; always effective
  ControlRequest r = blankReq(ControlRequest::FSM_CMD); r.cmd = Cmd::ESTOP;
  controlEnqueue(r);                    // full queue is fine: the flag is already set
  sendJson(200, "{\"ok\":true,\"queued\":true}");
}

// ---------------------------------------------------------------- setup
void webBegin() {
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/tof", HTTP_GET, handleTof);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/fsr/cal", HTTP_GET, handleCalGet);
  server.on("/api/fsr/cal", HTTP_POST, handleCalPost);
  server.on("/api/fsr/zero", HTTP_POST, handleZeroPost);
  server.on("/api/servo/jog", HTTP_POST, handleJog);
  server.on("/api/servo/stop", HTTP_POST, [] { simple(ControlRequest::JOG_STOP); });
  server.on("/api/servo/setOpen", HTTP_POST, [] { simple(ControlRequest::SET_OPEN); });
  server.on("/api/servo/setClose", HTTP_POST, [] { simple(ControlRequest::SET_CLOSE); });
  server.on("/api/servo/scan", HTTP_POST, [] { simple(ControlRequest::SCAN); });
  server.on("/api/servo/clearLimits", HTTP_POST, [] { simple(ControlRequest::CLEAR_LIMITS); });
  server.on("/api/servo/home", HTTP_POST, [] { simple(ControlRequest::HOME); });
  server.on("/api/servo/sweep", HTTP_POST, [] { simple(ControlRequest::SWEEP); });
  server.on("/api/sweep", HTTP_GET, handleSweepGet);
  server.on("/api/servo/positionMode", HTTP_POST, [] { simple(ControlRequest::SET_POSITION_MODE); });
  server.on("/api/servo/calibrateMiddle", HTTP_POST, [] { simple(ControlRequest::CALIBRATE_MIDDLE); });
  server.on("/api/fsm/arm", HTTP_POST, [] { fsmCmd(Cmd::ARM); });
  server.on("/api/fsm/disarm", HTTP_POST, [] { fsmCmd(Cmd::DISARM); });
  server.on("/api/fsm/open", HTTP_POST, [] { fsmCmd(Cmd::OPEN); });
  server.on("/api/fsm/estop", HTTP_POST, handleEstop);
  server.on("/api/fsm/reset", HTTP_POST, [] { fsmCmd(Cmd::RESET); });
  server.onNotFound([] { sendError(404, "not found"); });
  server.begin();
}

void webTask(void*) {
  for (;;) {
    server.handleClient();
    vTaskDelay(2);
  }
}
