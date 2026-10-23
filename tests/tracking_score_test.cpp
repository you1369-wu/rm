#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "task.h"
#include "tracking_mode/tracking_control/tracking_mode.hpp"
#include "tracking_mode/tracking_score/tracking_score.hpp"
#include "tracking_mode/tracking_score/tracking_score_task.h"

// Host-only scheduler/control dependencies; task registration remains in CubeMX.
TickType_t xTaskGetTickCount() { return 0; }
namespace tracking { Telemetry read_telemetry() { return {}; } }

bool near(float a, float b) { return std::fabs(a - b) < 1e-6f; }

void organizer_tests()
{
  yaw_detect(0, 0, nullptr);
  pitch_detect(0, 0, nullptr);
  float yaw = 0, pitch = 0;
  yaw_detect(0, 0, &yaw);
  pitch_detect(0, 0, &pitch);
  assert(near(yaw, .001f) && near(pitch, .001f));
  yaw_detect(.5f / 57.3f, 0, &yaw);  // Equality is a miss, as in the source module.
  pitch_detect(1.5f / 57.3f, 0, &pitch);
  assert(near(yaw, .000999f) && near(pitch, .000999f));
  yaw = pitch = 0;
  yaw_detect(1.0f / 57.3f, 0, &yaw);
  pitch_detect(1.0f / 57.3f, 0, &pitch);
  assert(yaw == 0 && near(pitch, .001f));
  yaw = 0;
  for (int i = 0; i < 1000; ++i) yaw_detect(0, 0, &yaw);
  assert(std::fabs(yaw - (1.0f - std::pow(.999f, 1000))) < .00001f);
}

void integration_tests()
{
  using namespace tracking_score;
  tracking::Telemetry t{};
  update_score(t, 0);
  assert(read_score().samples == 0 && !read_score().active);
  t.state = tracking::State::Running;
  t.run_started_ms = t.stamp_ms = 10;
  t.score_actual_yaw = .02f;  // Encoder yaw matches, but IMU yaw must determine the score.
  t.actual_yaw = t.actual_pitch = t.target_yaw = t.target_pitch = 0;
  update_score(t, 10);
  auto score = read_score();
  assert(score.samples == 1 && score.active);
  assert(score.yaw_ratio == 0 && near(score.pitch_ratio, .001f));
  update_score(t, 10);
  assert(read_score().samples == 1 && !read_score().active);
  ++t.stamp_ms;
  t.score_actual_yaw = 0;
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 2 && near(read_score().yaw_ratio, .001f));

  ++t.stamp_ms;
  update_score(t, t.stamp_ms + 2);  // Stale snapshots must not inflate the score.
  assert(read_score().samples == 2 && !read_score().active);
  t.score_actual_yaw = std::numeric_limits<float>::quiet_NaN();
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 2 && !read_score().active);
  t.score_actual_yaw = 0;
  t.target_pitch = std::numeric_limits<float>::infinity();
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 2);
  t.target_pitch = 0;

  const auto before = read_score();
  t.state = tracking::State::Fault;
  t.fault = tracking::Fault::ImuLost;
  update_score(t, t.stamp_ms);
  score = read_score();
  assert(!score.active && score.samples == before.samples);
  assert(score.yaw_ratio == before.yaw_ratio && score.pitch_ratio == before.pitch_ratio);
  t.state = tracking::State::Running;
  t.fault = tracking::Fault::None;
  t.run_started_ms = ++t.stamp_ms;
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 1 && near(read_score().yaw_ratio, .001f));
  // A restart is recognized even when the scoring task did not observe the exit state.
  t.run_started_ms = ++t.stamp_ms;
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 1 && near(read_score().pitch_ratio, .001f));
  ++t.stamp_ms;
  t.phase_ms = 0;  // Repeating trajectory phase does not reset a scoring session.
  update_score(t, t.stamp_ms);
  assert(read_score().samples == 2 && near(read_score().pitch_ratio, .001999f));

  t.run_started_ms = t.stamp_ms = UINT32_MAX;
  update_score(t, UINT32_MAX);
  t.stamp_ms = 0;
  update_score(t, 0);
  assert(read_score().samples == 2 && read_score().active);
  t.state = tracking::State::Disabled;
  update_score(t, 1);
  assert(read_score().samples == 2 && !read_score().active);
}

int main()
{
  organizer_tests();
  integration_tests();
  std::puts("PASS scoring: official thresholds/EMA, IMU yaw, lifecycle, freshness, NaN, tick wrap");
}
