// Control task: ties sensors, FSM and servo together. Owns all servo bus traffic.
#pragma once
#include <stdint.h>
#include "config_core.h"
#include "fsm_core.h"

struct ControlRequest {
  enum Kind : uint8_t { FSM_CMD, JOG, JOG_STOP, SET_OPEN, SET_CLOSE, SCAN, CONFIG, CLEAR_LIMITS, CALIBRATE_MIDDLE, SET_POSITION_MODE } kind;
  Cmd cmd; int16_t speed; bool ignoreLimits; GripConfig cfg;   // ignoreLimits: JOG only
};

struct ControlStatus {
  State state; State faultFrom; bool armed; bool openedSinceBoot;
  char reason[96]; char faultReason[96];
  Blocker blockers[Fsm::MAX_REPORTED]; uint8_t nBlockers;   // every blocker the FSM can report
  char lastActionMsg[96]; bool lastActionOk;
  uint32_t loopMaxUs;     // longest control-loop iteration in the last 5 s (diagnostics)
  uint32_t posJumps;      // servo position jumps detected since boot
};

void controlBegin();                              // creates the request queue (8); call before tasks
bool controlEnqueue(const ControlRequest& r);     // 0 wait; false when full
void controlRequestEstop();                       // checked first every iteration -> Cmd::ESTOP
ControlStatus controlGet();
GripConfig controlGetConfig();                    // copy of the active config (any task)
void controlTask(void*);
