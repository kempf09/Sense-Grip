#include "servo_proto.h"

size_t svBuildPacket(uint8_t id, uint8_t instr, const uint8_t* params, uint8_t n, uint8_t* out) {
  const uint8_t len = (uint8_t)(n + 2);
  uint8_t sum = (uint8_t)(id + len + instr);
  out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = len; out[4] = instr;
  for (uint8_t i = 0; i < n; i++) { out[5 + i] = params[i]; sum = (uint8_t)(sum + params[i]); }
  out[5 + n] = (uint8_t)~sum;
  return (size_t)n + 6;
}

SvParse svParseReply(const uint8_t* buf, size_t len, uint8_t wantId, uint8_t& status,
                     const uint8_t*& data, uint8_t& n, size_t& consumed) {
  consumed = 0; n = 0; data = nullptr;
  size_t h = 0; bool found = false;
  for (; h + 1 < len; h++) {
    if (buf[h] == 0xFF && buf[h + 1] == 0xFF) { found = true; break; }
  }
  if (!found) {
    // Keep a trailing 0xFF: it may be the first half of a header.
    consumed = (len > 0 && buf[len - 1] == 0xFF) ? len - 1 : len;
    return SvParse::NEED_MORE;
  }
  if (len < h + 4) { consumed = h; return SvParse::NEED_MORE; }
  const uint8_t id = buf[h + 2], plen = buf[h + 3];
  if (plen < 2) { consumed = h + 2; return SvParse::BAD_CHECKSUM; }
  const size_t total = (size_t)plen + 4;  // FF FF id len + (status + params + checksum)
  if (len < h + total) { consumed = h; return SvParse::NEED_MORE; }
  uint8_t sum = 0;
  for (size_t i = h + 2; i < h + total - 1; i++) sum = (uint8_t)(sum + buf[i]);
  if ((uint8_t)~sum != buf[h + total - 1]) { consumed = h + 2; return SvParse::BAD_CHECKSUM; }
  consumed = h + total;
  if (id != wantId) return SvParse::WRONG_ID;
  status = buf[h + 4];
  n = (uint8_t)(plen - 2);
  data = buf + h + 5;
  return SvParse::OK;
}

uint16_t svEncodeSM15(int32_t v) {
  uint32_t m = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  if (m > 0x7FFF) m = 0x7FFF;
  return (uint16_t)(m | (v < 0 ? 0x8000 : 0));
}
int32_t svDecodeSM15(uint16_t raw) {
  return (raw & 0x8000) ? -(int32_t)(raw & 0x7FFF) : (int32_t)raw;
}
int16_t svDecodeSM10(uint16_t raw) {
  return (raw & 0x400) ? (int16_t)-(int16_t)(raw & 0x3FF) : (int16_t)raw;
}

void svStatusText(uint8_t bits, char* out, size_t cap) {
  if (cap == 0) return;
  static const struct { uint8_t bit; const char* name; } names[] = {
    {0, "voltage"}, {1, "angle sensor"}, {2, "temperature"}, {3, "current"}, {5, "overload"}};
  size_t pos = 0; out[0] = '\0';
  for (const auto& e : names) {
    if (!(bits & (1u << e.bit))) continue;
    const char* parts[2] = { pos ? ", " : "", e.name };
    for (const char* p : parts)
      for (; *p && pos + 1 < cap; p++) out[pos++] = *p;
    out[pos] = '\0';
  }
}

static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

ServoFeedback svDecodeFeedback(const uint8_t b[18]) {
  ServoFeedback f;
  f.position = svDecodeSM15(rd16(b));
  f.speed = (int16_t)svDecodeSM15(rd16(b + 2));
  f.load = svDecodeSM10(rd16(b + 4)) * 0.001f;
  f.voltage = b[6] * 0.1f;
  f.temp = b[7];
  f.status = b[9];
  f.moving = (b[10] & 1) != 0;
  f.current = (int16_t)svDecodeSM15(rd16(b + 13));
  return f;
}
