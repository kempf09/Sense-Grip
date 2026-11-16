#include "check.h"
#include "contact_core.h"
#include <cstring>

TEST(contact_first_kind_latches) {
  FirstContact f[3] = {FirstContact::NONE, FirstContact::NONE, FirstContact::NONE};
  bool prox[3] = {false, false, false}, dist[3] = {true, false, false};
  updateFirstContact(f, prox, dist);
  CHECK(f[0] == FirstContact::DISTAL);
  prox[0] = true;
  updateFirstContact(f, prox, dist);
  CHECK(f[0] == FirstContact::DISTAL);                      // latched
  bool p2[3] = {false, true, false}, d2[3] = {false, true, false};
  updateFirstContact(f, p2, d2);
  CHECK(f[1] == FirstContact::BOTH);
  CHECK(f[2] == FirstContact::NONE);
}
TEST(contact_pattern_rules) {
  FirstContact a[3] = {FirstContact::PROXIMAL, FirstContact::DISTAL, FirstContact::NONE};
  CHECK(patternOf(a) == Pattern::WRAP);
  FirstContact b[3] = {FirstContact::DISTAL, FirstContact::NONE, FirstContact::DISTAL};
  CHECK(patternOf(b) == Pattern::PINCH);
  FirstContact c[3] = {FirstContact::BOTH, FirstContact::DISTAL, FirstContact::DISTAL};
  CHECK(patternOf(c) == Pattern::MIXED);
  FirstContact d[3] = {FirstContact::NONE, FirstContact::NONE, FirstContact::NONE};
  CHECK(patternOf(d) == Pattern::NONE);
}
TEST(contact_pattern_consistency) {
  CHECK(patternConsistent(Pattern::WRAP, ObjClass::LARGE));
  CHECK(!patternConsistent(Pattern::PINCH, ObjClass::LARGE));
  CHECK(patternConsistent(Pattern::PINCH, ObjClass::SMALL));
  CHECK(!patternConsistent(Pattern::WRAP, ObjClass::SMALL));
  CHECK(patternConsistent(Pattern::PINCH, ObjClass::DELICATE));
  CHECK(patternConsistent(Pattern::WRAP, ObjClass::MEDIUM));
  CHECK(patternConsistent(Pattern::PINCH, ObjClass::MEDIUM));
  CHECK(patternConsistent(Pattern::NONE, ObjClass::LARGE));
  CHECK(std::strcmp(patternName(Pattern::WRAP), "wrap") == 0);
}
