#include "app_state.hpp"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "task.h"

sp::DBus remote(&huart3);

RcState get_rc_state(uint32_t now_ms)
{
  RcState state = {};

  taskENTER_CRITICAL();
  state.sw_r = remote.sw_r;
  state.sw_l = remote.sw_l;
  state.alive = remote.is_alive(now_ms);
  taskEXIT_CRITICAL();

  return state;
}

extern "C" void uart_task(void const * argument)
{
  (void)argument;

  remote.request();

  while (true) {
    osDelay(10U);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  const uint32_t stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}