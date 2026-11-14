#include "json_util.h"

size_t jsonEscape(const char* in, char* out, size_t cap) {
  if (cap == 0) return 0;
  size_t n = 0;
  for (; *in; in++) {
    unsigned char c = (unsigned char)*in;
    char tmp[6]; size_t len;
    if (c == '"' || c == '\\') { tmp[0] = '\\'; tmp[1] = (char)c; len = 2; }
    else if (c < 0x20) {
      static const char hex[] = "0123456789abcdef";
      tmp[0] = '\\'; tmp[1] = 'u'; tmp[2] = '0'; tmp[3] = '0';
      tmp[4] = hex[c >> 4]; tmp[5] = hex[c & 15]; len = 6;
    } else { tmp[0] = (char)c; len = 1; }
    if (n + len + 1 > cap) break;
    for (size_t i = 0; i < len; i++) out[n++] = tmp[i];
  }
  out[n] = '\0';
  return n;
}
