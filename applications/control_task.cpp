#include "control_task.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"
#include "task.h"
#include "tracking_mode.hpp"
extern sp::CAN can1;
extern sp::RM_Motor motor_a, motor_b, motor_c;
GimbalData motor_a_data{}, motor_b_data{};

// 单一发送者；按发送ID分组，未使用槽位清零，不能将C610和GM6020拼成一帧。
static void send_commands(const tracking::Command & c)
{
  motor_a.cmd(c.torque_a);
  motor_b.cmd(c.torque_b);
  motor_c.cmd(c.torque_c);
  sp::RM_Motor * motors[] = {&motor_a, &motor_b, &motor_c};
  for (unsigned i = 0; i < 3; ++i) {
    bool sent = false;
    for (unsigned j = 0; j < i; ++j)
      if (motors[j]->tx_id == motors[i]->tx_id) sent = true;
    if (sent) continue;
    std::memset(can1.tx_data, 0, 8);
    for (auto * motor : motors)
      if (motor->tx_id == motors[i]->tx_id) motor->write(can1.tx_data);
    can1.send(motors[i]->tx_id);
  }
}
extern "C" void control_task(void const * argument)
{
  (void)argument;
  osDelay(100);
  can1.config();
  can1.start();
  osDelay(100);
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous = wake - tracking_config::kControlPeriodMs;
  for (;;) {
    RemoteTaskData remote{};
    taskENTER_CRITICAL();
    const bool ready = remote_task_get_data(&remote);
    const uint32_t now = xTaskGetTickCount();
    const tracking::Feedback f{
      motor_a.angle,
      motor_a.speed,
      motor_b.angle,
      motor_b.speed,
      motor_a.is_alive(now) && motor_b.is_alive(now) && motor_c.is_alive(now),
      motor_c.angle,
      motor_c.speed};
    taskEXIT_CRITICAL();
    const bool online = ready && remote.is_alive;
    const bool down = online && remote.sw_r == REMOTE_SWITCH_DOWN;
    const bool run = online && remote.sw_r == REMOTE_SWITCH_UP && remote.sw_l == REMOTE_SWITCH_UP;
    const auto c = tracking::update(run, down, online, f, now, now - previous);
    previous = now;
    send_commands(c);
    if (xTaskGetTickCount() - wake >= pdMS_TO_TICKS(tracking_config::kControlPeriodMs))
      wake = xTaskGetTickCount();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(tracking_config::kControlPeriodMs));
  }
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  const auto now = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();
    if (can1.frame_type) continue;
    if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, now);
    if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, now);
    if (can1.rx_id == motor_c.rx_id) motor_c.read(can1.rx_data, now);
  }
}
