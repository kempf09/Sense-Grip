#include "check.h"

int main() {
  int passed = 0, failed = 0;
  for (auto& t : chk::tests()) {
    chk::current() = t.name;
    int before = chk::failures();
    t.fn();
    if (chk::failures() == before) passed++; else failed++;
  }
  std::printf("%d passed, %d failed\n", passed, failed);
  return failed ? 1 : 0;
}
