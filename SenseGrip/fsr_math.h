// FSR voltage -> resistance -> force math and contact debouncing. Pure C++.
#pragma once
#include <stdint.h>

struct FsrCal { float fixedOhms, f1, o1, f2, o2, zeroMv; };
bool  fsrCalValid(const FsrCal& c);
float fsrOhms(float mv, float fixedOhms);   // 10e6 when mv<1, 0 when mv>=3299
float fsrForce(const FsrCal& c, float ohms); // 0 when ohms<=0||>=10e6; NAN when cal invalid

struct ContactDebounce { uint8_t count[3]; bool stable[3]; };
// A finger is "touching" when either of its channels (FINGER_FSR_MAP) is >= thresholdN.
void updateContacts(ContactDebounce& d, const float force[6], float thresholdN, uint8_t samples);

// Copies force[] to out[], with channels whose bit is clear in enabledMask set to 0
// (a disabled FSR never counts as contact, grasp force or force-limit input).
void maskForces(const float force[6], uint8_t enabledMask, float out[6]);

struct FsrAggregate { float avgContactForce; uint8_t channelsInContact; float maxForce; float fingerForce[3]; };
FsrAggregate aggregate(const float force[6], float thresholdN);
