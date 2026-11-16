// Control task: ties sensors, FSM and servo together. Owns all servo bus traffic.
#pragma once
#include <stdint.h>
#include "config_core.h"
#include "fsm_core.h"

struct ControlRequest {
  enum Kind : uint8_t { FSM_CMD, JOG, JOG_STOP, SET_OPEN, SET_CLOSE, SCAN, CONFIG, CLEAR_LIMITS, CALIBRATE_MIDDLE, SET_POSITION_MODE, HOME, SWEEP } kind;
  Cmd cmd; int16_t speed; bool ignoreLimits; GripConfig cfg;   // ignoreLimits: JOG only
  bool newFrame;   // SET_OPEN/SET_CLOSE while not homed: confirmed start of a new frame (drops the touch reference)
};

struct ControlStatus {
  State state; State faultFrom; bool armed; bool openedSinceBoot;
  char reason[96]; char faultReason[96];
  Blocker blockers[Fsm::MAX_REPORTED]; uint8_t nBlockers;   // every blocker the FSM can report
  char lastActionMsg[96]; bool lastActionOk;
  uint32_t loopMaxUs;     // longest control-loop iteration in the last 5 s (diagnostics)
  uint32_t posJumps;      // servo position jumps detected since boot
  bool homed, homing; int32_t homeResidual; char homeMsg[96];
  bool sweeping, sweepLoaded; int32_t sweepN;
  bool estValid; int32_t estPos, estTurn;   // SEN0628 power-up estimate (only before homing)
  uint32_t fsrSampleUsMax; // longest FSR sample pass in the last 5 s (spec: reduce oversampling if > 4 ms)
};

void controlBegin();                              // creates the request queue (8); call before tasks
bool controlEnqueue(const ControlRequest& r);     // 0 wait; false when full
void controlRequestEstop();                       // checked first every iteration -> Cmd::ESTOP
ControlStatus controlGet();
GripConfig controlGetConfig();                    // copy of the active config (any task)
void controlTask(void*);
