#ifndef CAN_TASK_HPP
#define CAN_TASK_HPP

#include <cstdint>

#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

inline constexpr uint8_t kMotorAId = 4U;
inline constexpr uint8_t kMotorBId = 2U;
inline constexpr float kMotorARatio = 1.0f;
inline constexpr float kMotorBRatio = 1.0f;

extern sp::CAN can1;
extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

void can_task_init();
void send_motor_commands();

#endif  // CAN_TASK_HPP
