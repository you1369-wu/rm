
#include "imu_task.h"

#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "imu_support.hpp"
#include "task.h"

// ======================================================
// 1. Cortex Live Watch 调试变量
// ======================================================
// 在 VS Code 的 Cortex Live Watch 中直接输入变量名即可监视。
// pitch、yaw、roll 单位：rad（弧度）
// gyro 单位：rad/s
// acc 单位：m/s²
//
// extern "C" 保证变量名称不经过 C++ 名称修饰。
// volatile 使这些调试变量的读写保留在内存中。

extern "C" {

volatile float debug_imu_pitch = 0.0f;
volatile float debug_imu_yaw = 0.0f;
volatile float debug_imu_roll = 0.0f;

volatile float debug_acc_x = 0.0f;
volatile float debug_acc_y = 0.0f;
volatile float debug_acc_z = 0.0f;

volatile float debug_gyro_x = 0.0f;
volatile float debug_gyro_y = 0.0f;
volatile float debug_gyro_z = 0.0f;

volatile uint32_t debug_imu_count = 0;
volatile uint32_t debug_imu_loop_count = 0;

volatile bool debug_imu_valid = false;
volatile bool debug_bmi_update_ok = false;

volatile float debug_imu_norm2 = 0.0f;
volatile uint32_t debug_imu_gap = 0;
}

// ======================================================
// 2. IMU 对象及数据缓存
// ======================================================

namespace
{

// C 板 IMU 坐标变换矩阵
const float rotation[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};

// BMI088
app::Bmi088WithStatus bmi088(
  &hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, rotation);

// Mahony 姿态解算
app::TimedMahony imu;
static_assert(configTICK_RATE_HZ == 1000, "IMU task intervals require a 1 ms RTOS tick");

// 最新 IMU 数据
ImuTaskData latest{};

// 数据是否有效
bool ready = false;

// 发布 IMU 数据
void publish(bool valid, const ImuTaskData & data)
{
  taskENTER_CRITICAL();

  ready = valid;

  if (valid) {
    latest = data;
  }

  taskEXIT_CRITICAL();
}

}  // namespace

// ======================================================
// 3. 提供给其他任务读取 IMU 数据的接口
// ======================================================

extern "C" bool imu_task_get_data(ImuTaskData * data)
{
  if (!data) {
    return false;
  }

  taskENTER_CRITICAL();

  const bool valid = ready;
  *data = latest;

  taskEXIT_CRITICAL();

  return valid;
}

// ======================================================
// 4. FreeRTOS IMU 任务
// ======================================================

extern "C" void imu_task(void const * argument)
{
  (void)argument;

  // 上电后等待硬件稳定
  osDelay(100);

  // 初始化 BMI088
  bmi088.init();

  TickType_t wake = xTaskGetTickCount();
  TickType_t previous = wake - 1;

  // ==================================================
  // 主循环
  // ==================================================
  for (;;) {
    // 记录 IMU 任务循环次数
    debug_imu_loop_count++;

    // ----------------------------------------------
    // 4.1 读取 BMI088 数据
    // ----------------------------------------------

    bmi088.update();

    // 实时记录加速度
    debug_acc_x = bmi088.acc[0];
    debug_acc_y = bmi088.acc[1];
    debug_acc_z = bmi088.acc[2];

    // 实时记录角速度
    debug_gyro_x = bmi088.gyro[0];
    debug_gyro_y = bmi088.gyro[1];
    debug_gyro_z = bmi088.gyro[2];

    // BMI088 更新状态
    debug_bmi_update_ok = bmi088.last_update_ok();

    // ----------------------------------------------
    // 4.2 计算任务运行间隔
    // ----------------------------------------------

    const TickType_t now = xTaskGetTickCount();

    const TickType_t gap = now - previous;

    previous = now;

    debug_imu_gap = static_cast<uint32_t>(gap);

    // ----------------------------------------------
    // 4.3 检查数据有效性
    // ----------------------------------------------

    const float norm2 =
      bmi088.acc[0] * bmi088.acc[0] + bmi088.acc[1] * bmi088.acc[1] + bmi088.acc[2] * bmi088.acc[2];

    debug_imu_norm2 = norm2;

    bool valid = bmi088.last_update_ok() && std::isfinite(norm2) && norm2 > 1.0f &&
                 std::isfinite(bmi088.gyro[0]) && std::isfinite(bmi088.gyro[1]) &&
                 std::isfinite(bmi088.gyro[2]);

    // ----------------------------------------------
    // 4.4 Mahony 姿态解算
    // ----------------------------------------------

    if (valid) {
      // 根据实际任务周期更新 dt
      valid = imu.set_dt(static_cast<float>(gap) * 0.001f);

      // 更新姿态
      if (valid) valid = imu.update(bmi088.acc, bmi088.gyro);
    }
    debug_imu_valid = valid;

    if (valid) {
      // ------------------------------------------
      // 4.5 更新 Cortex Live Watch 调试变量
      // ------------------------------------------

      debug_imu_roll = imu.roll;
      debug_imu_pitch = imu.pitch;
      debug_imu_yaw = imu.yaw;

      // 记录有效更新次数
      debug_imu_count++;

      // ------------------------------------------
      // 4.6 整理 IMU 数据
      // ------------------------------------------

      ImuTaskData data{};

      // 加速度
      data.acc_mps2 = {bmi088.acc[0], bmi088.acc[1], bmi088.acc[2]};

      // 角速度
      data.gyro_rad_s = {bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]};

      // 欧拉角
      data.euler = {imu.roll, imu.pitch, imu.yaw};

      // pitch 角速度
      data.pitch_rate_rad_s =
        bmi088.gyro[1] * std::cos(imu.roll) - bmi088.gyro[2] * std::sin(imu.roll);

      // 温度
      data.temp_c = bmi088.temp;

      // 时间戳
      data.stamp_ms = now;

      // 判断姿态角是否为有限数值
      const bool finite =
        std::isfinite(imu.roll) && std::isfinite(imu.pitch) && std::isfinite(imu.yaw);

      // 发布数据
      publish(finite, data);
    }
    else {
      // 当前 IMU 数据无效
      publish(false, {});
    }

    // ----------------------------------------------
    // 4.7 FreeRTOS 周期调度
    // ----------------------------------------------

    // if (xTaskGetTickCount() - wake >= pdMS_TO_TICKS(1)) {
    //   wake = xTaskGetTickCount();
    // }

    osDelay(1);
  }
}
