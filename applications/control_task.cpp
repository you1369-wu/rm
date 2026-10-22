//负责can的收发和电机控制
#include "control_task.hpp"

#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "fold_mode/fold_mode.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"

extern sp::CAN can1;
extern sp::RM_Motor motor_a;  //云台折叠机构
extern sp::RM_Motor motor_b;  //云台yaw轴
extern volatile uint16_t distance;

GimbalData motor_a_data;
GimbalData motor_b_data;

ControlMode control_mode = ControlMode::DISABLED;

//顶层状态机：右下失能，右中折叠，右上加左上循迹
static ControlMode select_control_mode(
  const RemoteTaskData & remote_data, bool remote_ready, bool motors_ready)
{
  // 公共安全条件优先；遥控器或电机反馈异常时，不允许任何运动模式输出。
  if (
    !remote_ready || !remote_data.is_alive || !motors_ready || !std::isfinite(motor_a.angle) ||
    !std::isfinite(motor_a.speed) || !std::isfinite(motor_b.angle) ||
    !std::isfinite(motor_b.speed)) {
    return ControlMode::DISABLED;
  }

  if (remote_data.sw_r == REMOTE_SWITCH_MID && fold_mode_ready()) {
    return ControlMode::FOLD;
  }
  if (remote_data.sw_r == REMOTE_SWITCH_UP && remote_data.sw_l == REMOTE_SWITCH_UP) {
    return ControlMode::TRACKING;
  }
  return ControlMode::DISABLED;
}

void run_disable_mode()
{
  //下档、失联、折叠超时或限位故障均发送两台电机的零力矩。
  clear_fold_controllers();
  motor_a_data.given_torque = 0.0f;
  motor_b_data.given_torque = 0.0f;
  motor_a_data.target_speed_set = 0.0f;
  motor_b_data.target_speed_set = 0.0f;
}

// 运动和失能共用发送出口；折叠模块只更新给定力矩。
static void send_motor_commands()
{
  motor_a.cmd(motor_a_data.given_torque);
  motor_b.cmd(motor_b_data.given_torque);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  // 软件零力矩依赖 CAN 正常发送，不能替代硬件急停；当前 send() 不反馈发送结果。
  can1.send(motor_a.tx_id);
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

  while (true) {
    RemoteTaskData remote_data = {};
    const bool remote_ready = remote_task_get_data(&remote_data);
    const uint32_t now_ms = osKernelSysTick();
    const bool motors_ready = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

    const ControlMode next_mode = select_control_mode(remote_data, remote_ready, motors_ready);
    if (next_mode != control_mode) {
      // 仅切换模式时通知折叠模块退出；先记录失联故障，并中断尚未完成的下档标定。
      update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);

      // 模式切换先清理旧控制输出和积分，不能把上一模式的力矩带入下一模式。
      run_disable_mode();
      control_mode = next_mode;
    }

    switch (control_mode) {
      case ControlMode::DISABLED:
        // 下档也必须更新折叠模块，保留开机标定及故障确认流程。
        update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);
        break;

      case ControlMode::FOLD:
        update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms);

        // 折叠过程中发生故障后，本周期即退回失能；故障仍由折叠模块锁存。
        if (!fold_mode_ready()) {
          update_fold_mode(remote_data, remote_ready, motors_ready, distance, now_ms, false);
          control_mode = ControlMode::DISABLED;
        }

        break;

      case ControlMode::TRACKING:
        // main 尚未接入循迹模块，此处只保持公共失能，不再更新折叠模块。
        run_disable_mode();
        break;
    }

    send_motor_commands();

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      // 在CAN接收中断中更新电机反馈, control_task使用最新反馈计算PID
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
