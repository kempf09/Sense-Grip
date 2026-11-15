// CF35-12 servo verbs over the half-duplex bus. All UART calls must come from one
// task (the control task); servoGet() is safe from any task/core.
#pragma once
#include <stdint.h>
#include "servo_proto.h"

struct ServoSnapshot {
  bool online; uint32_t ageMs; uint8_t mode; bool modeKnown; bool fullAngle;
  ServoFeedback fb;      // fb.position is LOGICAL (servo position + offset)
  int32_t rawPosition;   // position as reported by the servo
  uint32_t busErrors;    // replies that timed out / failed checksum since boot (diagnostics)
  uint8_t replyStatus;   // status byte of the latest reply (same bits as fb.status); 0 = none
  char lastError[64];    // last failure; cleared when a verb succeeds
};

void servoBegin(uint8_t id, uint32_t baud);
bool servoPoll();                                   // reads 56..73, updates snapshot
bool servoReadModeAndPhase();                       // regs 33, 18
bool servoRunAt(int16_t speedClosingPositive, uint16_t torqueLimit);
bool servoHoldHere(uint16_t torqueLimit, uint16_t speed);
bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit);
bool servoStop();                                   // torque limit 0, then torque off; true = torque off acked
bool servoCalibrateMiddle();                        // CalibrationOfs: present position becomes servo middle
bool servoJogTo(int32_t target, uint16_t speed, uint16_t torqueLimit); // position-mode jog step
bool servoSetPositionMode();                        // operator: torque off, EEPROM unlock, mode 0, lock
bool servoBootStop();
// Logical frame: logical = servo position + offset. Goals passed to verbs are logical.
void servoSetOffset(int32_t offset);
int32_t servoGetOffset();                               // same as servoStop (kept for the boot path)
bool servoScanBaud(uint32_t& foundBaud);            // tries {1000000,500000,250000,128000,115200,76800,57600,38400}
ServoSnapshot servoGet();
void servoSetLimits(int32_t closeLimit, int32_t openZero, bool closePositive);   // targets for servoRunAt
