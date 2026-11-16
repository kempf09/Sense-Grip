#include "plan_core.h"

PlanResult buildPlan(const GripConfig& cfg, bool homed, const SweepModel* m, const SweepSample* s,
                     const ObjectMeasure& obj, Override ov, bool delicate, int32_t position) {
  PlanResult r{};
  const bool sweep = m && s && m->n > 0;
  const bool measured = homed && sweep && obj.valid;
  r.detectedValid = measured;
  r.detected = measured ? classify(obj.widthMm, cfg.smallMaxMm, cfg.mediumMaxMm) : ObjClass::MEDIUM;
  ObjClass cls = r.detected;
  if (delicate) cls = ObjClass::DELICATE;
  else if (ov != Override::AUTO) cls = (ObjClass)((uint8_t)ov - 1);
  r.plan.cls = cls;
  r.plan.prof = profileOf(cfg, cls);
  r.plan.valid = measured || ov != Override::AUTO || delicate;
  r.plan.approach = measured;
  r.plan.approachGoal = position;
  if (measured) {
    const float target = obj.widthMm / 2 + r.plan.prof.standoffMm + 0.1f * obj.rangeMm;
    int32_t goal = positionForOpening(*m, s, position, target);
    if (cfg.closePositive ? goal > cfg.closeLimit : goal < cfg.closeLimit) goal = cfg.closeLimit;
    r.plan.approachGoal = goal;
  }
  if (!r.plan.valid) r.why = !homed ? "not homed" : !sweep ? "no sweep stored (run the calibration sweep)"
                                                         : "object not measured (no points in view)";
  return r;
}
