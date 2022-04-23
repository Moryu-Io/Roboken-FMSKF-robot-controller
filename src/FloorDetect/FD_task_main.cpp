#include <FreeRTOS_TEENSY4.h>

#include "FD_task_main.hpp"

#include "../Utility/util_led.hpp"

namespace FDT {

void prepare_task() {
}

void main(void *params) {

  while(1) {
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
    UTIL::toggle_LED0();
  }
}

}; // namespace FDT
