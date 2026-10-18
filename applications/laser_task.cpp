
#include "cmsis_os.h"
#include "io/laser/laser.hpp"

sp::LaserGY56 laser(&huart6);
volatile uint16_t distance = 0xFFFF;

extern "C" void laser_task(void const * argument)
{
  (void)argument;

  // 4.4.7-1: 独立任务接收测距；折叠状态机与电机命令由 control_task 负责。
  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    // 4.4.7-1: 仅发布未超时的有效距离；0xFFFF 表示本轮无可用测距。
    distance = laser.is_valid() ? laser.get_distance() : 0xFFFF;
    osDelay(1);
  }
}
