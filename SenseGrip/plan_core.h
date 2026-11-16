// Grasp plan (spec §7.3/7.4): effective class, profile and fast-approach goal from the smoothed
// SEN0628 measurement, the sweep opening table and the operator's override / delicate switches.
// Pure C++.
#pragma once
#include <stdint.h>
#include "config_core.h"
#include "fsm_core.h"
#include "object_core.h"
#include "sweep_core.h"

enum class Override : uint8_t { AUTO = 0, SMALL, MEDIUM, LARGE };
struct PlanResult { GraspPlan plan; ObjClass detected; bool detectedValid; const char* why; };

// Class: DELICATE if delicate, else the override, else classify(width). valid when the object was
// measured (homed, sweep, obj.valid) or an override / delicate is set. The fast-approach goal
// (only when measured) is where the fingertip opening = width/2 + stand-off + 10 % of the object
// range, never beyond closeLimit and never toward open from `position`.
// m / s null when no sweep is stored. why: reason when !plan.valid (else nullptr).
PlanResult buildPlan(const GripConfig& cfg, bool homed, const SweepModel* m, const SweepSample* s,
                     const ObjectMeasure& smoothedObj, Override ov, bool delicate, int32_t position);
