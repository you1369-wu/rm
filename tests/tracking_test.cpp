#include "tracking_controller.hpp"
#include "tracking_mode.hpp"
#include "trajectory_runtime.hpp"
#include "task.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>

namespace
{
uint32_t tick = 0;
unsigned remaining = 0;
std::function<void()> control;
struct EndRun {};
bool near(float a, float b, float tolerance = 1e-5f) { return std::fabs(a - b) < tolerance; }
void run_ticks(unsigned count, std::function<void()> function)
{
  remaining = count;
  control = function;
  try { trajectory_task(nullptr); }
  catch (const EndRun &) {}
}
tracking::Feedback feedback_at(float yp, float pp, float yv = 0, float pv = 0)
{
  return {tracking_config::kMotorAZero + yp, yv,
          tracking_config::kMotorBZero + pp, pv, true};
}
void acknowledge()
{
  tracking::update(false, true, true, feedback_at(0, 0), tick, 1);
  assert(tracking::read_telemetry().state == tracking::State::Disabled);
}
void assert_zero(tracking::Command c) { assert(c.torque_a == 0 && c.torque_b == 0); }
void test_controller()
{
  using namespace tracking;
  AxisController c(tracking_config::kYaw);
  auto out = c.step({0.1f, 2.0f, 0}, 0.1f, 2.0f, 0.001f);
  assert(near(out.torque, 0)); // exact velocity feedforward cancels steady tracking error
  out = c.step({100, 100, 0}, 0, 0, 0.001f);
  assert(near(out.speed_command, 10) && near(out.torque, 0.3f) && out.saturated);
  out = c.step({-100, -100, 0}, 0, 0, 0.001f);
  assert(near(out.torque, -0.3f));
  auto gains = tracking_config::kYaw;
  gains.angle_kp = gains.speed_kp = 0;
  gains.speed_ki = 10;
  gains.max_torque = 0.1f;
  gains.max_integral = 1;
  AxisController integral(gains);
  for (int i = 0; i < 10000; ++i) integral.step({0, 1, 0}, 0, 0, 0.001f);
  assert(integral.integral() <= 0.10001f); // no windup at the FINAL actuator limit
  const float before = integral.integral();
  integral.step({0, -1, 0}, 0, 0, 0.001f);
  assert(integral.integral() < before);
  assert(near(integral.step({NAN, 0, 0}, 0, 0, 0.001f).torque, 0));
  assert(integral.integral() == 0);
  gains.speed_ki = 0;
  gains.acceleration_ff = 0.01f;
  AxisController ff(gains);
  assert(near(ff.step({0, 0, 100}, 0, 0, 0.001f).torque, 0.1f));
  assert(near(c.step({0, 0, 0}, 0, 0, 0.006f).torque, 0));
  const auto first = approach(0.2f, -0.5f, 0, 2);
  const auto last = approach(0.2f, -0.5f, 3, 2);
  assert(near(first.position, 0.2f) && near(first.velocity, 0) && near(first.acceleration, 0));
  assert(near(last.position, -0.5f) && near(last.velocity, 0) && near(last.acceleration, 0));
  for (int i = 0; i <= 2000; ++i) {
    const auto r = approach(0.2f, -0.5f, i * 0.001f, 2);
    assert(r.position >= -0.50001f && r.position <= 0.20001f);
    assert(r.velocity <= 0.00001f);
  }
  std::puts("PASS controller: feedforward, limits, anti-windup, invalid input, approach");
}
void test_table()
{
  float peak_yv = 0, peak_pv = 0;
  for (uint32_t i = 0; i < TRAJECTORY_PERIOD_MS; ++i) {
    const auto a = trajectory_sample_at_ms(i);
    const auto b = trajectory_sample_at_ms(i + TRAJECTORY_PERIOD_MS);
    assert(near(a.yaw, b.yaw) && near(a.pitch, b.pitch));
    assert(std::fabs(a.yaw) <= 0.393f && std::fabs(a.pitch) <= 0.524f);
    assert(std::fabs(a.yaw_acc) <= 110 && std::fabs(a.pitch_acc) <= 70);
    peak_yv = std::fmax(peak_yv, std::fabs(a.yaw_vel));
    peak_pv = std::fmax(peak_pv, std::fabs(a.pitch_vel));
    const auto next = trajectory_sample_at_ms(i + 1);
    assert(std::fabs((next.yaw - a.yaw) * 1000 - a.yaw_vel) < 0.06f);
    assert(std::fabs((next.pitch - a.pitch) * 1000 - a.pitch_vel) < 0.04f);
  }
  assert(near(peak_yv, 7.148648f) && near(peak_pv, 8.308258f));
  std::printf("PASS trajectory table: period=%u ms, peak speeds=%.6f / %.6f rad/s\n",
              TRAJECTORY_PERIOD_MS, peak_yv, peak_pv);
}
void run_ideal(bool wrap_tick)
{
  tick = wrap_tick ? UINT32_MAX - 2300U : 0;
  acknowledge();
  bool saw_running = false;
  uint32_t running_samples = 0;
  run_ticks(3500, [&] {
    const auto p = trajectory_snapshot().point;
    const auto c = tracking::update(true, false, true,
      feedback_at(p.yaw, p.pitch, p.yaw_vel, p.pitch_vel), tick, 1);
    assert(std::fabs(c.torque_a) <= 0.3f && std::fabs(c.torque_b) <= 0.3f);
    const auto t = tracking::read_telemetry();
    assert(t.state != tracking::State::Fault);
    if (t.state == tracking::State::Running) {
      if (!saw_running) assert(t.phase_ms == 0); // playback starts at the first table point
      saw_running = true;
      ++running_samples;
      assert(t.rms_yaw < 1e-5f && t.rms_pitch < 1e-5f);
    }
    const auto legacy = trajectory_read();
    assert(near(legacy.yaw, p.yaw) && near(legacy.pitch, p.pitch));
  });
  assert(saw_running && running_samples > 1000);
  std::puts(wrap_tick ? "PASS RTOS tick rollover and playback" :
                        "PASS alignment, playback handshake, snapshots and RMS");
}
void test_faults()
{
  using namespace tracking;
  // State is Running after run_ideal. Loss must latch, and a mode change cannot clear it.
  assert_zero(update(false, false, false, feedback_at(0, 0), tick, 1));
  assert(read_telemetry().fault == Fault::RemoteLost);
  assert_zero(update(false, false, true, feedback_at(0, 0), tick, 1));
  assert(read_telemetry().state == State::Fault);
  acknowledge();
  run_ticks(1, [&] {
    auto f = feedback_at(0, 0);
    f.alive = false;
    assert_zero(update(true, false, true, f, tick, 1));
    assert(read_telemetry().fault == Fault::FeedbackLost);
  });
  acknowledge();
  run_ticks(1, [&] {
    assert_zero(update(true, false, true, feedback_at(0, 0), tick, 6));
    assert(read_telemetry().fault == Fault::Timing);
  });
  acknowledge();
  tick += 20;
  assert_zero(update(true, false, true, feedback_at(0, 0), tick, 1));
  assert(read_telemetry().fault == Fault::ReferenceStale);
  acknowledge();
  run_ticks(1, [&] {
    assert_zero(update(true, false, true, feedback_at(0.5f, 0), tick, 1));
    assert(read_telemetry().fault == Fault::TravelLimit);
  });
  acknowledge();
  run_ticks(1, [&] {
    assert_zero(update(true, false, true, feedback_at(NAN, 0), tick, 1));
    assert(read_telemetry().fault == Fault::NonFinite);
  });
  acknowledge();
  run_ticks(6100, [&] { update(true, false, true, feedback_at(0, 0), tick, 1); });
  assert(read_telemetry().fault == Fault::AlignTimeout);
  acknowledge();
  run_ticks(2700, [&] {
    const auto snap = trajectory_snapshot();
    float measured = snap.point.yaw;
    if (read_telemetry().state == State::Running)
      measured -= measured >= 0 ? 0.36f : -0.36f;
    update(true, false, true, feedback_at(measured, snap.point.pitch,
           snap.point.yaw_vel, snap.point.pitch_vel), tick, 1);
  });
  assert(read_telemetry().fault == Fault::TrackingError);
  acknowledge();
  std::puts("PASS faults: remote, motor, timing, stale reference, limits, NaN, alignment, error");
}
}
TickType_t xTaskGetTickCount() { return tick; }
void vTaskDelayUntil(TickType_t * previous, TickType_t increment)
{
  control();
  tick = *previous + increment;
  *previous = tick;
  if (--remaining == 0) throw EndRun{};
}
int main()
{
  test_controller();
  test_table();
  // Booting directly in trajectory mode must not actuate.
  assert_zero(tracking::update(true, false, true, feedback_at(0, 0), tick, 1));
  assert(tracking::read_telemetry().fault == tracking::Fault::NotArmed);
  run_ideal(false);
  run_ideal(true);
  test_faults();
  std::puts("All tracking tests passed. Hardware dynamics and ISR scheduling are not simulated.");
}
