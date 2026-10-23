//负责can的收发和电机控制
#include "control_task.hpp"

#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "fold_mode/fold_mode.hpp"
#include "imu/imu_task.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"
#include "tracking_mode/tracking_control/tracking_mode.hpp"
#include "tracking_mode/tracking_control/tracking_request.h"

extern sp::CAN can1;
extern sp::RM_Motor motor_a;  //云台折叠机构
extern sp::RM_Motor motor_b;  //云台yaw轴
extern sp::RM_Motor motor_c;  //顶部 M2006，实物 CAN ID 3
extern volatile uint16_t distance;

GimbalData motor_a_data;
GimbalData motor_b_data;
GimbalData motor_c_data;

ControlMode control_mode = ControlMode::DISABLED;

extern "C" {
// 保留循迹调试变量名：这里的 A 指循迹 yaw（ID 1，即 main 的 motor_b）。
volatile float debug_motor_a_angle = 0.0f;
volatile float debug_motor_a_speed = 0.0f;
volatile bool debug_motor_a_alive = false;
volatile float debug_motor_a_command = 0.0f;
volatile uint32_t debug_can_a_tx_id = 0;
volatile uint32_t debug_can_a_send_count = 0;
}

namespace
{
bool automatic_request = false, stop_latched = false;
}

extern "C" void gimbal_tracking_request_start(void)
{
  taskENTER_CRITICAL();
  automatic_request = true;
  taskEXIT_CRITICAL();
}

extern "C" void gimbal_tracking_request_stop(void)
{
  taskENTER_CRITICAL();
  automatic_request = false;
  stop_latched = true;
  taskEXIT_CRITICAL();
}

//顶层状态机：右下失能，右中折叠，右上加左上循迹
static ControlMode select_control_mode(
  const RemoteTaskData & remote_data, bool remote_ready, bool motors_ready,
  bool tracking_requested)
{
  // 公共安全条件优先；遥控器或电机反馈异常时，不允许任何运动模式输出。
  if (
    !remote_ready || !remote_data.is_alive || !motors_ready || !std::isfinite(motor_a.angle) ||
    !std::isfinite(motor_a.speed) || !std::isfinite(motor_b.angle) ||
    !std::isfinite(motor_b.speed)) {
    return ControlMode::DISABLED;
  }

  // 模式切换不清除另一模式的故障；两边均由右下档确认。
  if (fold_mode_faulted() || tracking::read_telemetry().fault != tracking::Fault::None ||
      stop_latched) return ControlMode::DISABLED;

  if (tracking_requested) {
    return ControlMode::TRACKING;
  }
  if (remote_data.sw_r == REMOTE_SWITCH_MID && fold_mode_ready()) {
    return ControlMode::FOLD;
  }
  return ControlMode::DISABLED;
}

void run_disable_mode()
{
  //下档、失联、任一模式故障均清零三台电机；循迹 PID 由模式退出接口单独清理。
  clear_fold_controllers();
  motor_a_data.given_torque = 0.0f;
  motor_b_data.given_torque = 0.0f;
  motor_c_data.given_torque = 0.0f;
  motor_a_data.target_speed_set = 0.0f;
  motor_b_data.target_speed_set = 0.0f;
  motor_c_data.target_speed_set = 0.0f;
}

// 运动和失能共用发送出口；折叠模块只更新给定力矩。
static void send_motor_commands()
{
  motor_a.cmd(motor_a_data.given_torque);
  motor_b.cmd(motor_b_data.given_torque);
  motor_c.cmd(motor_c_data.given_torque);
  debug_motor_a_command = motor_b_data.given_torque;
  debug_can_a_tx_id = motor_b.tx_id;
  sp::RM_Motor * motors[] = {&motor_a, &motor_b, &motor_c};
  // GM6020 电流帧与 M2006 控制帧分开发；同 ID 只发送一次，未使用槽位清零。
  for (unsigned i = 0; i < 3; ++i) {
    bool sent = false;
    for (unsigned j = 0; j < i; ++j)
      if (motors[j]->tx_id == motors[i]->tx_id) sent = true;
    if (sent) continue;
    for (auto & byte : can1.tx_data) byte = 0;
    for (auto * motor : motors)
      if (motor->tx_id == motors[i]->tx_id) motor->write(can1.tx_data);
    // 软件零力矩依赖 CAN 正常发送，不能替代硬件急停；当前 send() 不反馈发送结果。
    can1.send(motors[i]->tx_id);
    if (motors[i]->tx_id == motor_b.tx_id) ++debug_can_a_send_count;
  }
}

extern "C" void control_task(void const * argument)
{
  (void)argument;

  osDelay(100);

  // 4.4.7-1: 控制任务统一负责 CAN 和折叠/展开，测距由 laser_task 独立采集。
  can1.config();
  can1.start();
  osDelay(100);

  // 初始化
  motor_a_data.target_angle_set = motor_a.angle;
  motor_b_data.target_angle_set = motor_b.angle;
  motor_c_data.target_angle_set = motor_c.angle;
  uint32_t previous_ms = osKernelSysTick() - 1U;

  while (true) {
    RemoteTaskData remote_data = {};
    ImuTaskData imu_data = {};
    tracking::Feedback feedback{};
    taskENTER_CRITICAL();
    const bool remote_ready = remote_task_get_data(&remote_data);
    const uint32_t now_ms = osKernelSysTick();
    const bool motors_ready = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);
    // 按实物 ID 映射，而非照搬变量名：循迹 A=ID1，B=ID2，C=ID3。
    feedback.angle_a = motor_b.angle;
    feedback.speed_a = motor_b.speed;
    feedback.angle_b = motor_a.angle;
    feedback.speed_b = motor_a.speed;
    feedback.angle_c = motor_c.angle;
    feedback.speed_c = motor_c.speed;
    feedback.motors_alive = motors_ready && motor_c.is_alive(now_ms);
    feedback.imu_ready = imu_task_get_data(&imu_data);
    debug_motor_a_angle = motor_b.angle;
    debug_motor_a_speed = motor_b.speed;
    debug_motor_a_alive = motor_b.is_alive(now_ms);
    taskEXIT_CRITICAL();
    feedback.pitch = tracking_config::kImuPitchDirection * imu_data.euler.pitch_rad -
                     tracking_config::kImuPitchMountOffset;
    feedback.pitch_rate = tracking_config::kImuPitchDirection * imu_data.pitch_rate_rad_s;
    feedback.roll = imu_data.euler.roll_rad;
    feedback.imu_stamp_ms = imu_data.stamp_ms;
    const uint32_t gap_ms = now_ms - previous_ms;
    previous_ms = now_ms;
    const bool online = remote_ready && remote_data.is_alive;
    const bool down = online && remote_data.sw_r == REMOTE_SWITCH_DOWN;
    taskENTER_CRITICAL();
    if (!online || down) automatic_request = false;
    if (down) stop_latched = false;
    const bool auto_run = automatic_request, stopped = stop_latched;
    taskEXIT_CRITICAL();
    const bool tracking_requested = online && !down && !stopped &&
      (auto_run || (remote_data.sw_r == REMOTE_SWITCH_UP &&
                   remote_data.sw_l == REMOTE_SWITCH_UP));

    const ControlMode next_mode =
      select_control_mode(remote_data, remote_ready, motors_ready, tracking_requested);
    if (next_mode != control_mode) {
      if (control_mode == ControlMode::TRACKING) {
        // 仍请求循迹却因反馈异常退出时，先让循迹模块记录故障，再清理其独立控制器。
        if (tracking_requested)
          tracking::update(true, down, online, feedback, now_ms, gap_ms);
        tracking::update(false, down, online, feedback, now_ms, gap_ms);
      }
      // 仅切换模式时通知折叠模块退出；先记录失联故障，并中断尚未完成的下档标定。
      update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);

      // 模式切换先清理旧控制输出和积分，不能把上一模式的力矩带入下一模式。
      run_disable_mode();
      control_mode = fold_mode_faulted() ||
        tracking::read_telemetry().fault != tracking::Fault::None
        ? ControlMode::DISABLED : next_mode;
    }

    switch (control_mode) {
      case ControlMode::DISABLED:
        // 下档也必须更新折叠模块，保留开机标定及故障确认流程。
        update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);
        // 公共下档同时确认循迹故障/解锁，但不会运行循迹 PID。
        tracking::update(false, down, online, feedback, now_ms, gap_ms);
        break;

      case ControlMode::FOLD:
        // 未参与原折叠业务的顶部 M2006 保持零力矩；不要求其在线。
        motor_c_data.given_torque = 0.0f;
        update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms);

        // 折叠过程中发生故障后，本周期即退回失能；故障仍由折叠模块锁存。
        if (!fold_mode_ready()) {
          update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);
          control_mode = ControlMode::DISABLED;
        }

        break;

      case ControlMode::TRACKING:
      {
        // 循迹 PID/前馈全由 tracking_mode 自己计算，不叠加 fold_mode 的补偿。
        const auto command = tracking::update(true, down, online, feedback, now_ms, gap_ms);
        motor_b_data.given_torque = command.torque_a;  // yaw，ID 1
        motor_a_data.given_torque = command.torque_b;  // 大臂，ID 2
        motor_c_data.given_torque = command.torque_c;  // 顶部，ID 3
        if (tracking::read_telemetry().state == tracking::State::Fault) {
          tracking::update(false, down, online, feedback, now_ms, gap_ms);
          run_disable_mode();
          control_mode = ControlMode::DISABLED;
        }
        break;
      }
    }

    send_motor_commands();

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.frame_type) continue;

      // 在CAN接收中断中更新电机反馈, control_task使用最新反馈计算PID
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_c.rx_id) motor_c.read(can1.rx_data, stamp_ms);
    }
  }
}
