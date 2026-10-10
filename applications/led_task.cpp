#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  (void)argument;
  led.start();

  while (true) {
    led.set(0.0f, 0.0f, 1.0f);  // 蓝
    osDelay(300);

    led.set(0.0f, 1.0f, 0.0f);  // 绿
    osDelay(300);

    led.set(1.0f, 0.0f, 0.0f);  // 红
    osDelay(300);
  }
}
