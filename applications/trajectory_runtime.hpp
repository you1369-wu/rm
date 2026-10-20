#pragma once
#include "trajectory_task.hpp"

// Task-only API. Readers copy one coherent point and its timestamp.
struct TrajectorySnapshot
{
  TrajectoryPoint point;
  uint32_t stamp_ms;
  uint32_t phase_ms;
  bool running;
  bool ready;
};
void trajectory_set_running(bool running);
TrajectorySnapshot trajectory_snapshot();
