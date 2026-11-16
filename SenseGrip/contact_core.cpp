#include "contact_core.h"

void updateFirstContact(FirstContact f[3], const bool proximal[3], const bool distal[3]) {
  for (int i = 0; i < 3; i++) {
    if (f[i] != FirstContact::NONE) continue;
    if (proximal[i] && distal[i]) f[i] = FirstContact::BOTH;
    else if (proximal[i]) f[i] = FirstContact::PROXIMAL;
    else if (distal[i]) f[i] = FirstContact::DISTAL;
  }
}

Pattern patternOf(const FirstContact f[3]) {
  bool any = false, prox = false, allDistal = true;
  for (int i = 0; i < 3; i++) {
    if (f[i] == FirstContact::NONE) continue;
    any = true;
    if (f[i] == FirstContact::PROXIMAL) prox = true;
    if (f[i] != FirstContact::DISTAL) allDistal = false;
  }
  if (!any) return Pattern::NONE;
  if (prox) return Pattern::WRAP;
  return allDistal ? Pattern::PINCH : Pattern::MIXED;
}

bool patternConsistent(Pattern p, ObjClass c) {
  if (p == Pattern::NONE || p == Pattern::MIXED) return true;
  switch (c) {
    case ObjClass::LARGE: return p == Pattern::WRAP;
    case ObjClass::MEDIUM: return true;
    default: return p == Pattern::PINCH;
  }
}

const char* patternName(Pattern p) {
  switch (p) {
    case Pattern::WRAP: return "wrap";
    case Pattern::PINCH: return "pinch";
    case Pattern::MIXED: return "mixed";
    default: return "none";
  }
}
