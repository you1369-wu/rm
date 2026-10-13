
#include "cmsis_os.h"
#include "io/laser/laser.hpp"

sp::LaserGY56 laser(&huart6);
uint16_t distance = 0xFFFF;

extern "C" void laser_task(void const * argument)
{
  (void)argument;

  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    // 对外只发布未超时的有效距离, 否则发布无效值
    distance = laser.is_valid() ? laser.get_distance() : 0xFFFF;
    osDelay(1);
  }
}
