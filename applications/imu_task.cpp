#include "app_state.hpp"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"

namespace
{
const float kImuToBody[3][3] = {
  {0.0f, -1.0f, 0.0f},
  {1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};

constexpr float kMahonyDt = 1e-3f;
constexpr uint32_t kPlotDivider = 50U;

// huart1 已交给联动调试用的 plotter, 两者不能同时发送
constexpr bool kEnableImuPlot = false;

ImuState g_imu_state = {0.0f, 0.0f, 0U, false};
}  // namespace

extern "C"
{
volatile float g_imu_yaw_rad = 0.0f;
volatile float g_imu_vyaw_radps = 0.0f;
volatile uint8_t g_imu_ready = 0U;
}
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, kImuToBody);
sp::Mahony imu(kMahonyDt);
sp::Plotter imu_plotter(&huart1, false);
volatile uint32_t g_imu_uart_frame_count = 0U;

ImuState get_imu_state()
{
  ImuState state = {};

  taskENTER_CRITICAL();
  state = g_imu_state;
  taskEXIT_CRITICAL();

  return state;
}

extern "C" void imu_task(void const * argument)
{
  (void)argument;

  bmi088.init();

  uint32_t last_wake_ms = osKernelSysTick();
  uint32_t plot_counter = 0U;

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    const uint32_t now_ms = osKernelSysTick();

    taskENTER_CRITICAL();
    g_imu_state.yaw = imu.yaw;
    g_imu_state.vyaw = imu.vyaw;
    g_imu_state.stamp_ms = now_ms;
    g_imu_state.valid = true;
    g_imu_yaw_rad = imu.yaw;
    g_imu_vyaw_radps = imu.vyaw;
    g_imu_ready = 1U;
    taskEXIT_CRITICAL();

    if (kEnableImuPlot && ++plot_counter >= kPlotDivider) {
      imu_plotter.plot(
        bmi088.acc[0], bmi088.acc[1], bmi088.acc[2],
        bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],
        bmi088.temp);
      ++g_imu_uart_frame_count;
      plot_counter = 0U;
    }

    osDelayUntil(&last_wake_ms, 1U);
  }
}
