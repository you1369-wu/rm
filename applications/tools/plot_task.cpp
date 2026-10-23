#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "imu_task.h"
#include "io/plotter/plotter.hpp"
#include "task.h"

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;
  for (;;) {
    // imu 对象属于 IMU 任务内部，通过已有接口读取姿态，不能使用 extern 直接访问。
    ImuTaskData imu_data = {};
    taskENTER_CRITICAL();
    const bool ready = imu_task_get_data(&imu_data);
    taskEXIT_CRITICAL();
    if (ready) {
      // 三个通道依次为 yaw、pitch、roll，单位 rad。
      plotter.plot(imu_data.euler.yaw_rad, imu_data.euler.pitch_rad, imu_data.euler.roll_rad);
    }
    osDelay(10);
  }
}
