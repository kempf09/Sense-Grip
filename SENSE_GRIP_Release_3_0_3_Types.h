#pragma once
#include <stdint.h>

// Arduino's .ino preprocessor generates function prototypes before the .ino
// type definitions. These forward declarations make custom parameter and
// return types visible when those prototypes are compiled.
struct FsrConfig;
struct FsrReading;
struct ControlCommand;
struct Resp;

enum State : uint8_t;
enum MotionOwner : uint8_t;
enum ControlCommandType : uint8_t;
