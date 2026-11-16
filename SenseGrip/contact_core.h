// Contact pattern (spec §7.5): which pad of each finger touched first. Display only. Pure C++.
#pragma once
#include <stdint.h>
#include "config_core.h"

enum class FirstContact : uint8_t { NONE, PROXIMAL, DISTAL, BOTH };   // per finger, latched
enum class Pattern : uint8_t { NONE, WRAP, PINCH, MIXED };

// Latches the first contact kind per finger (unchanged once != NONE).
void updateFirstContact(FirstContact f[3], const bool proximal[3], const bool distal[3]);
// Any finger proximal-first -> WRAP; every touched finger distal-only first -> PINCH;
// none touched -> NONE; otherwise (both pads in the same sample) MIXED.
Pattern patternOf(const FirstContact f[3]);
// Expected: LARGE -> WRAP, SMALL/DELICATE -> PINCH, MEDIUM -> either. NONE and MIXED -> true.
bool patternConsistent(Pattern p, ObjClass c);
const char* patternName(Pattern p);
