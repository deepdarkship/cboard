#include "app_state.hpp"

#include <cmath>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"

namespace
{
  // ========================= 必须按实车修改的参数 =========================
  // 电机 CAN ID。若 A/B 使用同一控制帧，本程序会自动合成一帧发送。
  constexpr uint8_t kMotorAId = 4U;
  constexpr uint8_t kMotorBId = 2U;

  // 电机编码器到输出轴的真实机械减速比。GM6020 直驱时通常为 1.0f。
  constexpr float kMotorARatio = 1.0f;
  constexpr float kMotorBRatio = 1.0f;

  // 机械正方向系数。若给正扭矩时电机实际转向与期望相反，将对应值改为 -1.0f。
  constexpr float kMotorADir = 1.0f;
  constexpr float kMotorBDir = 1.0f;

  // R 标对齐时读取到的 RM_Motor::angle，单位 rad。必须用调试器实测后改写。
  constexpr float kAngleAR = 0.0f;
  constexpr float kAngleBR = 0.0f;

  // 左拨杆对应的 B/A 联动比例。最终实际符号会再乘上 kMotorBDir。
  constexpr float kLinkBDown = 0.5f;
  constexpr float kLinkBMid = -1.0f;
  constexpr float kLinkBUp = 3.0f;
  constexpr bool kLeftSwitchSwapUpDown = false;  // 实测左拨杆上下相反时置 true
  constexpr bool kEnableManualSource = true;     // 调试比例时可临时置 false

  // ========================= 控制与安全参数 =========================
  constexpr bool kEnableMotorOutput = true;

  constexpr float kControlDt = 1e-3f;
  constexpr float kTorqueLimitNm = 0.15f;
  constexpr float kPositionKp = 1.0f;
  constexpr float kPositionKd = 0.0f;

  constexpr float kManualDetectErrorRad = 0.08f;
  constexpr float kManualDetectSpeedRadps = 0.15f;
  constexpr float kManualReleaseSpeedRadps = 0.10f;
  constexpr uint32_t kManualHoldMs = 150U;
  constexpr uint32_t kManualArmDelayMs = 200U;

  constexpr float kYawMovingThresholdRadps = 0.05f;
  constexpr uint32_t kImuTimeoutMs = 50U;

  // 调试用上位机曲线发送周期(配合 1ms 控制周期)  
  constexpr uint32_t kPlotDivider = 50U;

  constexpr float kResetPositionTolRad = 0.02f;
  constexpr float kResetSpeedTolRadps = 0.15f;
  constexpr uint32_t kResetSettleMs = 100U;

  // A 电机与 C 板 yaw 保持 1:1 联动
  constexpr float kLinkA = kMotorADir * 1.0f;

  sp::CAN can1(&hcan1);
  sp::RM_Motor motor_a(kMotorAId, sp::RM_Motors::GM6020, kMotorARatio, true);
  sp::RM_Motor motor_b(kMotorBId, sp::RM_Motors::GM6020, kMotorBRatio, true);

  sp::PID pid_a(
      kControlDt, kPositionKp, 0.0f, kPositionKd, kTorqueLimitNm, 0.0f, 1.0f, false, false);
  sp::PID pid_b(
      kControlDt, kPositionKp, 0.0f, kPositionKd, kTorqueLimitNm, 0.0f, 1.0f, false, false);

  // 调试用上位机曲线(huart1)。注意: 同一串口只能有一个 sp::Plotter 在发送。
  sp::Plotter link_plotter(&huart1, false);

  enum class ManualSource
  {
    NONE,
    A,
    B
  };

  bool yaw_initialized = false;
  float yaw_prev = 0.0f;
  float yaw_cont = 0.0f;
  float yaw_last_cycle = 0.0f;
  float yaw_rate = 0.0f;

  bool reference_initialized = false;
  float a0 = 0.0f;
  float b0 = 0.0f;
  float yaw0 = 0.0f;
  float kB = kLinkBMid;

  bool left_initialized = false;
  sp::DBusSwitchMode last_left = sp::DBusSwitchMode::MID;

  ManualSource manual_source = ManualSource::NONE;
  bool manual_armed = false;
  uint32_t manual_armed_start_ms = 0U;
  uint32_t manual_last_motion_ms = 0U;
  bool reset_settle_active = false;
  uint32_t reset_settle_start_ms = 0U;

  float wrap_pi(float angle)
  {
    while (angle > sp::SP_PI)
      angle -= 2.0f * sp::SP_PI;
    while (angle < -sp::SP_PI)
      angle += 2.0f * sp::SP_PI;
    return angle;
  }

  float get_b_link_ratio(sp::DBusSwitchMode sw_l)
  {
    if (kLeftSwitchSwapUpDown)
    {
      if (sw_l == sp::DBusSwitchMode::DOWN)
        sw_l = sp::DBusSwitchMode::UP;
      else if (sw_l == sp::DBusSwitchMode::UP)
        sw_l = sp::DBusSwitchMode::DOWN;
    }

    float ratio = kLinkBMid;
    if (sw_l == sp::DBusSwitchMode::DOWN)
      ratio = kLinkBDown;
    if (sw_l == sp::DBusSwitchMode::UP)
      ratio = kLinkBUp;
    return kMotorBDir * ratio;
  }

  void update_yaw(const ImuState & state)
  {
    if (!yaw_initialized)
    {
      yaw_prev = state.yaw;
      yaw_cont = state.yaw;
      yaw_last_cycle = yaw_cont;
      yaw_initialized = true;
      return;
    }

    yaw_cont += wrap_pi(state.yaw - yaw_prev);
    yaw_prev = state.yaw;
    yaw_rate = (yaw_cont - yaw_last_cycle) / kControlDt;
    yaw_last_cycle = yaw_cont;
  }

  void rebase_reference(float a, float b, float yaw)
  {
    a0 = a;
    b0 = b;
    yaw0 = yaw;
    reference_initialized = true;
    manual_source = ManualSource::NONE;
    manual_armed = false;
    reset_settle_active = false;
  }

  void clear_controllers()
  {
    pid_a.clear();
    pid_b.clear();
  }

  void safe_stop()
  {
    clear_controllers();
    g_motor_a_torque_cmd = 0.0f;
    g_motor_b_torque_cmd = 0.0f;
    motor_a.cmd(0.0f);
    motor_b.cmd(0.0f);
    reference_initialized = false;
    left_initialized = false;
    manual_source = ManualSource::NONE;
    manual_armed = false;
    reset_settle_active = false;
  }

  void send_motor_commands()
  {
    uint8_t tx_data[8] = {};

    if (motor_a.tx_id == motor_b.tx_id)
    {
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

  void control_motor_a(float target, float target_speed, uint32_t now_ms)
  {
    if (!motor_a.is_alive(now_ms))
    {
      pid_a.clear();
      g_motor_a_torque_cmd = 0.0f;
      motor_a.cmd(0.0f);
      return;
    }

    pid_a.calc(target, motor_a.angle, target_speed, motor_a.speed);
    g_motor_a_torque_cmd = pid_a.out;
    motor_a.cmd(pid_a.out);
  }

  void control_motor_b(float target, float target_speed, uint32_t now_ms)
  {
    if (!motor_b.is_alive(now_ms))
    {
      pid_b.clear();
      g_motor_b_torque_cmd = 0.0f;
      motor_b.cmd(0.0f);
      return;
    }

    pid_b.calc(target, motor_b.angle, target_speed, motor_b.speed);
    g_motor_b_torque_cmd = pid_b.out;
    motor_b.cmd(pid_b.out);
  }
}  // namespace

extern "C"
{
  volatile float g_motor_a_angle_rad = 0.0f;
  volatile float g_motor_b_angle_rad = 0.0f;
  volatile float g_motor_a_speed_radps = 0.0f;
  volatile float g_motor_b_speed_radps = 0.0f;
  volatile uint8_t g_motor_a_online = 0U;
  volatile uint8_t g_motor_b_online = 0U;

  volatile float g_link_yaw_cont_rad = 0.0f;
  volatile float g_link_a_ref_rad = 0.0f;
  volatile float g_link_b_ref_rad = 0.0f;
  volatile uint8_t g_link_manual_source = 0U;

  volatile uint8_t g_cfg_motor_a_id = 0U;
  volatile uint8_t g_cfg_motor_b_id = 0U;
  volatile float g_cfg_motor_a_ratio = 0.0f;
  volatile float g_cfg_motor_b_ratio = 0.0f;
  volatile float g_cfg_motor_a_dir = 0.0f;
  volatile float g_cfg_motor_b_dir = 0.0f;
  volatile float g_cfg_angle_a_r = 0.0f;
  volatile float g_cfg_angle_b_r = 0.0f;
  volatile uint8_t g_cfg_enable_motor_output = 0U;

  volatile uint8_t g_rc_sw_r = 0U;
  volatile uint8_t g_rc_sw_l = 0U;
  volatile uint8_t g_rc_alive = 0U;

  volatile uint8_t g_link_output_enabled = 0U;
  volatile uint8_t g_link_imu_ok = 0U;
  volatile uint8_t g_link_motors_ok = 0U;
  volatile float g_link_yaw_delta_rad = 0.0f;
  volatile float g_link_kb = 0.0f;
  volatile uint8_t g_link_manual_armed = 0U;
  volatile float g_motor_a_torque_cmd = 0.0f;
  volatile float g_motor_b_torque_cmd = 0.0f;
}

// CAN1 RX0 中断入口。本项目 CubeMX 未生成该中断服务函数，
// 按中间件 io/can/readme.md 的中断接收方式在此补上。
extern "C" void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }

// 电机反馈帧的解析放在 CAN 接收中断回调中，与中间件示例保持一致。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;

  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    can1.recv();

    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}

extern "C" void linkage_task(void const *argument)
{
  (void)argument;

  can1.config();
  can1.start();
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

  g_cfg_motor_a_id = kMotorAId;
  g_cfg_motor_b_id = kMotorBId;
  g_cfg_motor_a_ratio = kMotorARatio;
  g_cfg_motor_b_ratio = kMotorBRatio;
  g_cfg_motor_a_dir = kMotorADir;
  g_cfg_motor_b_dir = kMotorBDir;
  g_cfg_angle_a_r = kAngleAR;
  g_cfg_angle_b_r = kAngleBR;
  g_cfg_enable_motor_output = kEnableMotorOutput ? 1U : 0U;

  uint32_t plot_counter = 0U;
  uint32_t last_wake_ms = osKernelSysTick();

  while (true)
  {
    const uint32_t now_ms = osKernelSysTick();

    g_motor_a_angle_rad = motor_a.angle;
    g_motor_b_angle_rad = motor_b.angle;
    g_motor_a_speed_radps = motor_a.speed;
    g_motor_b_speed_radps = motor_b.speed;
    g_motor_a_online = motor_a.is_alive(now_ms) ? 1U : 0U;
    g_motor_b_online = motor_b.is_alive(now_ms) ? 1U : 0U;
    g_link_yaw_cont_rad = yaw_cont;
    g_link_manual_source = static_cast<uint8_t>(manual_source);

    const ImuState imu_state = get_imu_state();
    const RcState rc_state = get_rc_state(now_ms);
    const bool imu_ok =
        imu_state.valid && ((now_ms - imu_state.stamp_ms) <= kImuTimeoutMs);
    const bool motors_ok = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

    g_rc_sw_r = static_cast<uint8_t>(rc_state.sw_r);
    g_rc_sw_l = static_cast<uint8_t>(rc_state.sw_l);
    g_rc_alive = rc_state.alive ? 1U : 0U;
    g_link_output_enabled = kEnableMotorOutput ? 1U : 0U;
    g_link_imu_ok = imu_ok ? 1U : 0U;
    g_link_motors_ok = motors_ok ? 1U : 0U;

    // 将联动/遥控关键量送上位机曲线, 便于观察是否随拨杆变化
    if (++plot_counter >= kPlotDivider) {
      plot_counter = 0U;
      link_plotter.plot(
        static_cast<float>(rc_state.sw_r), static_cast<float>(rc_state.sw_l),
        rc_state.alive ? 1.0f : 0.0f, g_link_kb, g_link_yaw_cont_rad,
        g_link_yaw_delta_rad, g_motor_a_angle_rad, g_motor_b_angle_rad,
        static_cast<float>(g_link_manual_source));
    }

    if (imu_ok)
    {
      update_yaw(imu_state);
    }
    else
    {
      yaw_initialized = false;
      yaw_rate = 0.0f;
    }

    if (!kEnableMotorOutput || !imu_ok || !rc_state.alive || !motors_ok)
    {
      safe_stop();
      send_motor_commands();
      osDelayUntil(&last_wake_ms, 1U);
      continue;
    }

    if (rc_state.sw_r == sp::DBusSwitchMode::DOWN)
    {
      safe_stop();
      send_motor_commands();
      osDelayUntil(&last_wake_ms, 1U);
      continue;
    }

    if (rc_state.sw_r == sp::DBusSwitchMode::MID)
    {
      if (!reference_initialized)
      {
        rebase_reference(motor_a.angle, motor_b.angle, yaw_cont);
      }

      if (!left_initialized)
      {
        last_left = rc_state.sw_l;
        left_initialized = true;
      }

      if (rc_state.sw_l != last_left)
      {
        // 档位切换时先以当前位置重基准，避免 B 电机因比例变化突然跳变。
        rebase_reference(motor_a.angle, motor_b.angle, yaw_cont);
        last_left = rc_state.sw_l;
      }

      kB = get_b_link_ratio(rc_state.sw_l);

      const float yaw_delta = yaw_cont - yaw0;
      g_link_yaw_delta_rad = yaw_delta;
      g_link_kb = kB;
      float a_target = a0 + kLinkA * yaw_delta;
      float b_target = b0 + kB * yaw_delta;
      float a_target_speed = kLinkA * yaw_rate;
      float b_target_speed = kB * yaw_rate;

      const float a_error = a_target - motor_a.angle;
      const float b_error = b_target - motor_b.angle;
      const bool c_board_moving = std::fabs(yaw_rate) > kYawMovingThresholdRadps;
      const bool motors_settled =
          std::fabs(a_error) < (kManualDetectErrorRad * 0.5f) &&
          std::fabs(b_error) < (kManualDetectErrorRad * 0.5f);

      if (!c_board_moving && motors_settled)
      {
        if (!manual_armed)
        {
          manual_armed = true;
          manual_armed_start_ms = now_ms;
        }
      }
      else
      {
        manual_armed = false;
      }

      const bool manual_ready =
          manual_armed && ((now_ms - manual_armed_start_ms) >= kManualArmDelayMs);
      g_link_manual_armed = manual_ready ? 1U : 0U;

      // 人手转动某一台电机时，把该电机视为输入端，另一台按比例跟随。
      if (kEnableManualSource && manual_source == ManualSource::NONE && manual_ready)
      {
        const bool a_moved =
            std::fabs(a_error) > kManualDetectErrorRad &&
            std::fabs(motor_a.speed) > kManualDetectSpeedRadps &&
            std::fabs(a_error) > std::fabs(b_error);
        const bool b_moved =
            std::fabs(b_error) > kManualDetectErrorRad &&
            std::fabs(motor_b.speed) > kManualDetectSpeedRadps &&
            std::fabs(b_error) > std::fabs(a_error);

        if (a_moved)
        {
          manual_source = ManualSource::A;
          manual_last_motion_ms = now_ms;
        }
        else if (b_moved)
        {
          manual_source = ManualSource::B;
          manual_last_motion_ms = now_ms;
        }
      }

      if (manual_source != ManualSource::NONE)
      {
        const float source_speed =
            (manual_source == ManualSource::A) ? motor_a.speed : motor_b.speed;

        if (std::fabs(source_speed) > kManualReleaseSpeedRadps)
        {
          manual_last_motion_ms = now_ms;
        }

        const bool released = (now_ms - manual_last_motion_ms) > kManualHoldMs;
        if (c_board_moving || released)
        {
          // 手动结束后以当前位置为新零点，C 板参考零点随 A 电机位置变化，
          // 再次转动 C 板时继续联动而不会自动回正。
          rebase_reference(motor_a.angle, motor_b.angle, yaw_cont);

          manual_source = ManualSource::NONE;
          manual_armed = false;
        }
      }

      if (manual_source == ManualSource::A)
      {
        // A 作为手动输入端，A 不施加位置保持扭矩，B 跟随 A。
        b_target = b0 + kB * (motor_a.angle - a0);
        b_target_speed = kB * motor_a.speed;
        pid_a.clear();
        motor_a.cmd(0.0f);
        control_motor_b(b_target, b_target_speed, now_ms);
      }
      else if (manual_source == ManualSource::B)
      {
        // B 作为手动输入端，B 不施加位置保持扭矩，A 跟随 B。
        a_target = a0 + (motor_b.angle - b0) / kB;
        a_target_speed = motor_b.speed / kB;
        pid_b.clear();
        motor_b.cmd(0.0f);
        control_motor_a(a_target, a_target_speed, now_ms);
      }
      else
      {
        control_motor_a(a_target, a_target_speed, now_ms);
        control_motor_b(b_target, b_target_speed, now_ms);
      }

      g_link_a_ref_rad = a_target;
      g_link_b_ref_rad = b_target;
      g_link_manual_source = static_cast<uint8_t>(manual_source);
      send_motor_commands();
      osDelayUntil(&last_wake_ms, 1U);
      continue;
    }

    if (rc_state.sw_r == sp::DBusSwitchMode::UP)
    {
      reference_initialized = false;
      left_initialized = false;
      manual_source = ManualSource::NONE;
      manual_armed = false;

      const float a_error = kAngleAR - motor_a.angle;
      const float b_error = kAngleBR - motor_b.angle;

      control_motor_a(kAngleAR, 0.0f, now_ms);
      control_motor_b(kAngleBR, 0.0f, now_ms);

      const bool in_position =
          std::fabs(a_error) < kResetPositionTolRad &&
          std::fabs(b_error) < kResetPositionTolRad &&
          std::fabs(motor_a.speed) < kResetSpeedTolRadps &&
          std::fabs(motor_b.speed) < kResetSpeedTolRadps;

      if (in_position)
      {
        if (!reset_settle_active)
        {
          reset_settle_active = true;
          reset_settle_start_ms = now_ms;
        }
        else if ((now_ms - reset_settle_start_ms) >= kResetSettleMs)
        {
          // 复位完成后以 R 标位置为新联动零点。
          rebase_reference(motor_a.angle, motor_b.angle, yaw_cont);
        }
      }
      else
      {
        reset_settle_active = false;
      }

      send_motor_commands();
      osDelayUntil(&last_wake_ms, 1U);
      continue;
    }

    safe_stop();
    send_motor_commands();
    osDelayUntil(&last_wake_ms, 1U);
  }
}
