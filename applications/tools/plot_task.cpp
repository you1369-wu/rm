#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "tracking_mode.hpp"

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const auto t = tracking::read_telemetry();
    if constexpr (tracking_config::kPlotPage == 0) {
      plotter.plot(
        t.target_yaw, t.actual_yaw, t.target_pitch, t.actual_pitch, t.error_yaw, t.error_pitch,
        t.arm_error, t.tip_position, t.torque_yaw, t.torque_arm, t.torque_pitch, t.rms_yaw,
        t.rms_pitch, static_cast<float>(t.state), static_cast<float>(t.fault),
        static_cast<float>(t.phase_ms), static_cast<float>(t.saturation_mask), t.imu_pitch);
    }
    else if constexpr (tracking_config::kPlotPage == 1) {
      plotter.plot(
        t.speed_command_yaw, t.speed_yaw, t.speed_command_pitch, t.speed_pitch, t.pid_yaw, t.ff_yaw,
        t.torque_yaw, t.pid_pitch, t.ff_pitch, t.torque_pitch, t.accel_yaw, t.accel_pitch,
        t.torque_arm, t.arm_error, static_cast<float>(t.state), static_cast<float>(t.fault),
        static_cast<float>(t.gap_ms), static_cast<float>(t.saturation_mask));
    }
    else {
      // Official channels: CH1 pitch_ratio, CH2 yaw_ratio. Both are [0,1].
      plotter.plot(t.pitch_ratio, t.yaw_ratio);
    }
    // Page 0/1: AA BB 48 + 18 floats; page 2: AA BB 08 + 2 floats.
    if (xTaskGetTickCount() - wake >= pdMS_TO_TICKS(tracking_config::kPlotPeriodMs))
      wake = xTaskGetTickCount();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(tracking_config::kPlotPeriodMs));
  }
}