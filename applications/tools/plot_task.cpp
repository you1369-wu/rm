#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "tracking_mode.hpp"

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;
  TickType_t wake_tick = xTaskGetTickCount();
  for (;;) {
    const auto t = tracking::read_telemetry();
    plotter.plot(
      t.target_yaw, t.actual_yaw, t.target_pitch, t.actual_pitch, t.target_arm, t.actual_arm,
      t.target_tip, t.actual_tip, t.torque_yaw, t.torque_arm, t.torque_pitch, t.rms_yaw,
      t.rms_pitch, static_cast<float>(t.state), static_cast<float>(t.fault),
      static_cast<float>(t.calibrated), static_cast<float>(t.phase_ms),
      static_cast<float>(t.saturation_mask));
    // 75字节*100Hz=7500B/s < 115200/10=11520B/s。
    if (xTaskGetTickCount() - wake_tick >= pdMS_TO_TICKS(tracking_config::kPlotPeriodMs))
      wake_tick = xTaskGetTickCount();
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(tracking_config::kPlotPeriodMs));
  }
}
