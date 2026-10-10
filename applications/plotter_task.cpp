#include "cmsis_os.h"
#include "app_state.hpp"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

extern sp::BMI088 bmi088;

namespace
{
constexpr bool kPlotGyro = false;  // false: 加速度三轴; true: 角速度三轴
constexpr uint32_t kPlotPeriodMs = 20U;
}

sp::Plotter axis_plotter(&huart1, false);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    if (g_imu_ready != 0U) {
      if (kPlotGyro) {
        axis_plotter.plot(bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
      } else {
        axis_plotter.plot(bmi088.acc[0], bmi088.acc[1], bmi088.acc[2]);
      }
    }

    osDelay(kPlotPeriodMs);
  }
}
