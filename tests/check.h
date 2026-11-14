// Minimal self-registering host test framework.
#pragma once
#include <cmath>
#include <cstdio>
#include <vector>

namespace chk {
struct Test { const char* name; void (*fn)(); };
inline std::vector<Test>& tests() { static std::vector<Test> t; return t; }
inline int& failures() { static int f = 0; return f; }
inline const char*& current() { static const char* c = ""; return c; }
struct Reg { Reg(const char* n, void (*f)()) { tests().push_back({n, f}); } };
inline void fail(const char* file, int line, const char* expr) {
  std::printf("FAIL [%s] %s:%d: %s\n", current(), file, line, expr);
  failures()++;
}
}  // namespace chk

#define TEST(name) \
  static void test_##name(); \
  static chk::Reg reg_##name(#name, test_##name); \
  static void test_##name()

#define CHECK(cond) do { if (!(cond)) chk::fail(__FILE__, __LINE__, #cond); } while (0)
#define CHECK_EQ(a, b) do { if (!((a) == (b))) chk::fail(__FILE__, __LINE__, #a " == " #b); } while (0)
#define CHECK_NEAR(a, b, eps) do { if (!(std::fabs((double)(a) - (double)(b)) <= (double)(eps))) chk::fail(__FILE__, __LINE__, "|" #a " - " #b "| <= " #eps); } while (0)
