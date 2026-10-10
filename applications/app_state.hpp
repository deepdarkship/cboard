#ifndef APP_STATE_HPP
#define APP_STATE_HPP

#include <cstdint>

#include "io/dbus/dbus.hpp"

struct ImuState
{
  float yaw;
  float vyaw;
  uint32_t stamp_ms;
  bool valid;
};

struct RcState
{
  sp::DBusSwitchMode sw_r;
  sp::DBusSwitchMode sw_l;
  bool alive;
};

ImuState get_imu_state();
RcState get_rc_state(uint32_t now_ms);

extern "C"
{
extern volatile float g_imu_yaw_rad;
extern volatile float g_imu_vyaw_radps;
extern volatile uint8_t g_imu_ready;

extern volatile float g_motor_a_angle_rad;
extern volatile float g_motor_b_angle_rad;
extern volatile float g_motor_a_speed_radps;
extern volatile float g_motor_b_speed_radps;
extern volatile uint8_t g_motor_a_online;
extern volatile uint8_t g_motor_b_online;

extern volatile float g_link_yaw_cont_rad;
extern volatile float g_link_a_ref_rad;
extern volatile float g_link_b_ref_rad;
extern volatile uint8_t g_link_manual_source;

extern volatile uint8_t g_cfg_motor_a_id;
extern volatile uint8_t g_cfg_motor_b_id;
extern volatile float g_cfg_motor_a_ratio;
extern volatile float g_cfg_motor_b_ratio;
extern volatile float g_cfg_motor_a_dir;
extern volatile float g_cfg_motor_b_dir;
extern volatile float g_cfg_angle_a_r;
extern volatile float g_cfg_angle_b_r;
extern volatile uint8_t g_cfg_enable_motor_output;

extern volatile uint8_t g_rc_sw_r;
extern volatile uint8_t g_rc_sw_l;
extern volatile uint8_t g_rc_alive;

extern volatile uint8_t g_link_output_enabled;
extern volatile uint8_t g_link_imu_ok;
extern volatile uint8_t g_link_motors_ok;
extern volatile float g_link_yaw_delta_rad;
extern volatile float g_link_kb;
extern volatile uint8_t g_link_manual_armed;
extern volatile float g_motor_a_torque_cmd;
extern volatile float g_motor_b_torque_cmd;
}

#endif  // APP_STATE_HPP