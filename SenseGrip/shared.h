// Cross-task shared-state helpers. One spinlock guards all snapshot structs.
#pragma once
#include <Arduino.h>

extern portMUX_TYPE gMux;

// Copies src under gMux. T must be trivially copyable.
template <class T>
T snap(const T& src) {
  T out;
  portENTER_CRITICAL(&gMux);
  memcpy(static_cast<void*>(&out), static_cast<const void*>(&src), sizeof(T));
  portEXIT_CRITICAL(&gMux);
  return out;
}
