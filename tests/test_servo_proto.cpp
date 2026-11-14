#include "check.h"
#include "servo_proto.h"
#include <cstring>

TEST(sv_build_ping) {
  uint8_t out[8];
  size_t n = svBuildPacket(1, SV_PING, nullptr, 0, out);
  const uint8_t want[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  CHECK_EQ(n, sizeof(want));
  CHECK(memcmp(out, want, sizeof(want)) == 0);
}

TEST(sv_parse_ok_and_checksum) {
  // id 1, len 4, status 0, params {0x12,0x34}; sum = 1+4+0+0x12+0x34 = 0x4B -> ~ = 0xB4
  uint8_t good[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x12, 0x34, 0xB4};
  uint8_t st = 9; const uint8_t* d = nullptr; uint8_t n = 0; size_t c = 0;
  CHECK(svParseReply(good, sizeof(good), 1, st, d, n, c) == SvParse::OK);
  CHECK_EQ(st, 0); CHECK_EQ(n, 2); CHECK_EQ(d[0], 0x12); CHECK_EQ(d[1], 0x34);
  CHECK_EQ(c, sizeof(good));
  CHECK(svParseReply(good, sizeof(good), 2, st, d, n, c) == SvParse::WRONG_ID);
  CHECK_EQ(c, sizeof(good));

  uint8_t bad[sizeof(good)]; memcpy(bad, good, sizeof(good)); bad[7] ^= 0xFF;
  CHECK(svParseReply(bad, sizeof(bad), 1, st, d, n, c) == SvParse::BAD_CHECKSUM);
  CHECK(c > 0);

  CHECK(svParseReply(good, sizeof(good) - 1, 1, st, d, n, c) == SvParse::NEED_MORE);
  CHECK_EQ(c, 0);

  uint8_t junk[3 + sizeof(good)] = {0x00, 0x55, 0xFE};
  memcpy(junk + 3, good, sizeof(good));
  CHECK(svParseReply(junk, sizeof(junk), 1, st, d, n, c) == SvParse::OK);
  CHECK_EQ(c, sizeof(junk));
  CHECK_EQ(d[0], 0x12);

  uint8_t onlyJunk[] = {0x01, 0x02};
  CHECK(svParseReply(onlyJunk, 2, 1, st, d, n, c) == SvParse::NEED_MORE);
  CHECK_EQ(c, 2);
}

TEST(sv_sm15_roundtrip) {
  const int32_t vals[] = {-1234, 0, 32767, -32767};
  for (int32_t v : vals) CHECK_EQ(svDecodeSM15(svEncodeSM15(v)), v);
  CHECK_EQ(svEncodeSM15(100000), 0x7FFF);
  CHECK_EQ(svEncodeSM15(-100000), 0xFFFF);
  CHECK_EQ(svDecodeSM10(0x400 | 5), -5);
}

TEST(sv_status_text) {
  char t[48];
  svStatusText(0x04, t, sizeof(t));
  CHECK(strstr(t, "temperature") != nullptr);
  svStatusText(0x10, t, sizeof(t));
  CHECK_EQ(t[0], '\0');
  svStatusText(0x21, t, sizeof(t));
  CHECK(strcmp(t, "voltage, overload") == 0);
  char s[6];
  svStatusText(0x2F, s, sizeof(s));
  CHECK_EQ(strlen(s), 5);
}

TEST(sv_decode_feedback) {
  uint8_t b[18] = {0};
  uint16_t p = svEncodeSM15(-100);
  b[0] = (uint8_t)p; b[1] = (uint8_t)(p >> 8);
  b[6] = 120; b[7] = 40; b[10] = 1;
  ServoFeedback f = svDecodeFeedback(b);
  CHECK_EQ(f.position, -100);
  CHECK_NEAR(f.voltage, 12.0, 0.001);
  CHECK_EQ(f.temp, 40);
  CHECK(f.moving);
}
