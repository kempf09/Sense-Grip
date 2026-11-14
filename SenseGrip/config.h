// Persistent configuration in NVS. Only this module touches namespaces sgcfg/fsm/cf35.
#pragma once
#include <stdint.h>
#include "config_core.h"

GripConfig configLoad();   // sgcfg, else legacy migration, else defaults; always clamped
bool configSave(const GripConfig& c);
void servoIdBaudLoad(uint8_t& id, uint32_t& baud);
bool servoIdBaudSave(uint8_t id, uint32_t baud);
