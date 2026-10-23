#include "control_task.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "imu_task.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"
#include "task.h"
#include "tracking_mode.hpp"
#include "tracking_request.h"
extern sp::CAN can1;
extern sp::RM_Motor motor_a, motor_b, motor_c;
extern "C" {
volatile float debug_motor_a_angle = 0.0f;
volatile float debug_motor_a_speed = 0.0f;
volatile bool debug_motor_a_alive = false;
volatile float debug_motor_a_command = 0.0f;
volatile uint32_t debug_can_a_tx_id = 0;
volatile uint32_t debug_can_a_send_count = 0;
}
GimbalData motor_a_data{}, motor_b_data{};
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
// Sole CAN command writer. GM6020 current-mode and C610 require separate frames.
static void send_commands(const tracking::Command & c)
{
  debug_motor_a_command = c.torque_a;
  debug_can_a_tx_id = motor_a.tx_id;
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
    for (auto * m : motors)
      if (m->tx_id == motors[i]->tx_id) m->write(can1.tx_data);
    can1.send(motors[i]->tx_id);
    if (motors[i]->tx_id == motor_a.tx_id) {
      debug_can_a_send_count++;
    }
  }
}
extern "C" void control_task(void const * argument)
{
  using namespace tracking_config;
  (void)argument;
  osDelay(100);
  can1.config();
  can1.start();
  osDelay(100);
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous = wake - 1;
  for (;;) {
    RemoteTaskData remote{};
    ImuTaskData imu{};
    tracking::Feedback f{};
    taskENTER_CRITICAL();
    const auto now = xTaskGetTickCount();
    const bool remote_ready = remote_task_get_data(&remote);
    f.angle_a = motor_a.angle;
    f.speed_a = motor_a.speed;

    debug_motor_a_angle = motor_a.angle;
    debug_motor_a_speed = motor_a.speed;
    debug_motor_a_alive = motor_a.is_alive(now);

    f.angle_b = motor_b.angle;
    f.speed_b = motor_b.speed;
    f.angle_c = motor_c.angle;
    f.speed_c = motor_c.speed;
    f.motors_alive = motor_a.is_alive(now) && motor_b.is_alive(now) && motor_c.is_alive(now);
    f.imu_ready = imu_task_get_data(&imu);
    taskEXIT_CRITICAL();
    f.pitch = kImuPitchDirection * imu.euler.pitch_rad - kImuPitchMountOffset;
    f.pitch_rate = kImuPitchDirection * imu.pitch_rate_rad_s;
    f.roll = imu.euler.roll_rad;
    f.imu_stamp_ms = imu.stamp_ms;
    const bool online = remote_ready && remote.is_alive;
    const bool down = online && remote.sw_r == REMOTE_SWITCH_DOWN;
    // Explicit request from the preceding stage OR both switches UP.
    // Right DOWN/offline always cancel automatic mode; stop requires DOWN before rearm.
    taskENTER_CRITICAL();
    if (!online || down) automatic_request = false;
    if (down) stop_latched = false;
    const bool auto_run = automatic_request, stopped = stop_latched;
    taskEXIT_CRITICAL();
    const bool requested =
      online && !down && !stopped &&
      (auto_run || (remote.sw_r == REMOTE_SWITCH_UP && remote.sw_l == REMOTE_SWITCH_UP));
    const auto c = tracking::update(requested, down, online, f, now, now - previous);
    previous = now;
    send_commands(c);
    if (xTaskGetTickCount() - wake >= pdMS_TO_TICKS(kControlPeriodMs)) wake = xTaskGetTickCount();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kControlPeriodMs));
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
