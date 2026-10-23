#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// CubeMX external task entry. Owns its 1 ms loop; do not wrap in another loop.
void tracking_score_task(void const * argument);

#ifdef __cplusplus
}

#include <cstdint>

namespace tracking { struct Telemetry; }
namespace tracking_score
{
struct Snapshot
{
  float pitch_ratio = 0, yaw_ratio = 0;  // Filtered hit ratios in [0, 1].
  uint32_t samples = 0, stamp_ms = 0;    // Accepted samples, last sample time.
  bool active = false;                  // Fresh Running data accepted this cycle.
};

// Input: one coherent control snapshot and current milliseconds. Output: stored ratios.
// Single writer: call only from tracking_score_task, once per 1 ms cycle.
void update_score(const tracking::Telemetry & telemetry, uint32_t now_ms);
// Output: coherent copy for plot_task/debug; no state changes.
Snapshot read_score();
}  // namespace tracking_score
#endif
