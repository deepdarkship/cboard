#include "can_task.hpp"

#include <cstring>

#include "cmsis_os.h"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(kMotorAId, sp::RM_Motors::GM6020, kMotorARatio, true);
sp::RM_Motor motor_b(kMotorBId, sp::RM_Motors::GM6020, kMotorBRatio, true);

namespace
{
void configure_filters()
{
  CAN_FilterTypeDef filter = {0};
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0xFFE0U;
  filter.FilterMaskIdLow = 0U;
  filter.SlaveStartFilterBank = 14U;

  filter.FilterBank = 0U;
  filter.FilterIdHigh = static_cast<uint16_t>((0x204U + kMotorAId) << 5U);
  can1.config(filter);

  filter.FilterBank = 1U;
  filter.FilterIdHigh = static_cast<uint16_t>((0x204U + kMotorBId) << 5U);
  can1.config(filter);
}
}

void can_task_init()
{
  configure_filters();
  can1.start();
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void send_motor_commands()
{
  uint8_t tx_data[8] = {0};

  if (motor_a.tx_id == motor_b.tx_id) {
    motor_a.write(tx_data);
    motor_b.write(tx_data);
    std::memcpy(can1.tx_data, tx_data, sizeof(tx_data));
    can1.send(motor_a.tx_id);
    return;
  }

  std::memset(can1.tx_data, 0, sizeof(can1.tx_data));
  motor_a.write(can1.tx_data);
  can1.send(motor_a.tx_id);

  std::memset(can1.tx_data, 0, sizeof(can1.tx_data));
  motor_b.write(can1.tx_data);
  can1.send(motor_b.tx_id);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;

  const uint32_t stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    can1.recv();
    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp_ms);
    } else if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
