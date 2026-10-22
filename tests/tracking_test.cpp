// CAN和电机对象定义
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tracking_config.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(tracking_config::kYawId, sp::RM_Motors::GM6020);  // A电机, CAN ID 1
sp::RM_Motor motor_b(tracking_config::kArmId, sp::RM_Motors::GM6020);  // B电机, CAN ID 2

sp::RM_Motor motor_c(tracking_config::kTipId, sp::RM_Motors::M2006, sp::M2006_P36, true);

extern "C" void can_task(void const * argument)
{
  (void)argument;

  while (true) {
    // 当前CAN收发由control_task统一负责, 保留空闲任务入口便于CubeMX配置回退
    osDelay(1000);
  }
}
