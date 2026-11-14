// Stub; replaced by the real web server in a later task.
#include <Arduino.h>
#include "web.h"

void webBegin() {}

void webTask(void*) {
  for (;;) vTaskDelay(1000);
}
