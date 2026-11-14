#include "check.h"
#include "json_util.h"
#include <cstring>

TEST(json_escape_plain_and_specials) {
  char o[64];
  size_t n = jsonEscape("abc", o, sizeof(o));
  CHECK_EQ(n, 3u); CHECK(strcmp(o, "abc") == 0);
  n = jsonEscape("a\"b\\c", o, sizeof(o));
  CHECK(strcmp(o, "a\\\"b\\\\c") == 0); CHECK_EQ(n, strlen(o));
  n = jsonEscape("x\n\t\x01y", o, sizeof(o));
  CHECK(strcmp(o, "x\\u000a\\u0009\\u0001y") == 0); CHECK_EQ(n, strlen(o));
}

TEST(json_escape_truncation_never_splits) {
  const char* in = "a\"b";            // full output: a \" b
  char o[16];
  for (size_t cap = 1; cap <= 6; cap++) {
    memset(o, 'Z', sizeof(o));
    size_t n = jsonEscape(in, o, cap);
    CHECK_EQ(o[n], '\0'); CHECK(n < cap);
    CHECK_EQ(n, strlen(o));
    // never ends with a dangling backslash (odd run of trailing backslashes)
    size_t bs = 0; while (bs < n && o[n - 1 - bs] == '\\') bs++;
    CHECK_EQ(bs % 2, 0u);
  }
  CHECK_EQ(jsonEscape(in, o, 3), 1u);   // "a" only: \" needs 2 + NUL
  CHECK_EQ(jsonEscape(in, o, 4), 3u);   // a\"
  CHECK_EQ(jsonEscape("\x02", o, 6), 0u);   // needs 6 + NUL = 7
  CHECK_EQ(jsonEscape("\x02", o, 7), 6u);
  CHECK_EQ(jsonEscape("abc", o, 0), 0u);
}
