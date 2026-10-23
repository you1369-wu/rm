#include "tracking_score_task.h"

#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "tracking_mode/tracking_control/tracking_mode.hpp"
#include "tracking_score.hpp"

static_assert(configTICK_RATE_HZ == 1000, "Scoring requires a 1 ms FreeRTOS tick");

namespace tracking_score
{
namespace
{
Snapshot current{}, published{};  // Task-owned accumulator and atomic reader snapshot.
uint32_t run_started_ms = 0;      // Distinguishes separate trajectory runs.
bool running = false, sampled = false;
}

void update_score(const tracking::Telemetry & t, uint32_t now_ms)
{
  current.active = false;
  if (t.state == tracking::State::Running && t.fault == tracking::Fault::None) {
    if (!running || run_started_ms != t.run_started_ms) {
      current = {};
      sampled = false;
      run_started_ms = t.run_started_ms;
    }
    running = true;
    // Never count an old control snapshot twice, or score invalid/stale angles as hits.
    if ((!sampled || current.stamp_ms != t.stamp_ms) &&
        now_ms - t.stamp_ms <= tracking_config::kControlPeriodMs &&
        std::isfinite(t.score_actual_yaw) && std::isfinite(t.actual_pitch) &&
        std::isfinite(t.target_yaw) && std::isfinite(t.target_pitch)) {
      yaw_detect(t.score_actual_yaw, t.target_yaw, &current.yaw_ratio);
      pitch_detect(t.actual_pitch, t.target_pitch, &current.pitch_ratio);
      if (current.samples < UINT32_MAX) ++current.samples;
      current.stamp_ms = t.stamp_ms;
      current.active = sampled = true;
    }
  }
  else {
    running = false;  // Retain the result until the next Running entry.
  }
  taskENTER_CRITICAL();
  published = current;
  taskEXIT_CRITICAL();
}

Snapshot read_score()
{
  taskENTER_CRITICAL();
  const auto result = published;
  taskEXIT_CRITICAL();
  return result;
}
}  // namespace tracking_score

extern "C" void tracking_score_task(void const * argument)
{
  (void)argument;
  for (;;) {
    const auto telemetry = tracking::read_telemetry();
    tracking_score::update_score(telemetry, xTaskGetTickCount());
    osDelay(1);
  }
}
