// CF35-12 servo verbs over the half-duplex bus. All UART calls must come from one
// task (the control task); servoGet() is safe from any task/core.
#pragma once
#include <stdint.h>
#include "servo_proto.h"

struct ServoSnapshot {
  bool online; uint32_t ageMs; uint8_t mode; bool modeKnown; bool fullAngle;
  ServoFeedback fb; char lastError[48];
};

void servoBegin(uint8_t id, uint32_t baud);
bool servoPoll();                                   // reads 56..73, updates snapshot
bool servoReadModeAndPhase();                       // regs 33, 18
bool servoRunAt(int16_t speedClosingPositive, uint16_t torqueLimit);
bool servoHoldHere(uint16_t torqueLimit, uint16_t speed);
bool servoMoveTo(int32_t position, uint16_t speed, uint16_t torqueLimit);
bool servoStop();                                   // torque off
bool servoScanBaud(uint32_t& foundBaud);            // tries {1000000,500000,250000,128000,115200,76800,57600,38400}
ServoSnapshot servoGet();
void servoSetRunViaPosition(bool v, int32_t closeLimit, int32_t openZero, bool closePositive);
