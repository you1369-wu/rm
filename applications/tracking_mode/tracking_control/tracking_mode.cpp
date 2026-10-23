#include "tracking_mode.hpp"

#include <algorithm>
#include <cmath>

#include "FreeRTOS.h"
#include "task.h"
#include "tracking_mode/tracking_generation/trajectory_runtime.hpp"

extern "C" {
volatile int debug_tracking_state = 0;
volatile int debug_tracking_fault = 0;
volatile bool debug_tracking_calibrated = false;

volatile float debug_tracking_target_pitch = 0;
volatile float debug_tracking_actual_pitch = 0;
volatile float debug_tracking_error_pitch = 0;

volatile float debug_tracking_arm_error = 0;
volatile float debug_tracking_yaw = 0;

volatile unsigned int debug_tracking_phase_ms = 0;

volatile float debug_tracking_target_yaw = 0.0f;
volatile float debug_tracking_speed_yaw = 0.0f;
volatile float debug_tracking_speed_pitch = 0.0f;

volatile float debug_tracking_torque_yaw = 0.0f;
volatile float debug_tracking_speed_command_yaw = 0.0f;
volatile float debug_tracking_actual_speed_yaw = 0.0f;

volatile float debug_tracking_error_yaw = 0.0f;
}

namespace tracking
{
namespace
{
using namespace tracking_config;
AxisController yaw_control(kYaw), arm_control(kArm), tip_control(kTip);
Telemetry t{}, published{};
bool armed = false, warm = false, settling = false, large_error = false, arm_drift = false;
uint32_t stage_ms = 0, warm_ms = 0, settled_ms = 0, error_ms = 0, arm_drift_ms = 0;
float c_entry = 0, level_start = 0;
double square_yaw = 0, square_pitch = 0;
bool inside(float q, float lo, float hi) { return q >= lo && q <= hi; }
void reset()
{
  yaw_control.reset();
  arm_control.reset();
  tip_control.reset();
  trajectory_set_running(false);
  warm = settling = large_error = arm_drift = false;
}
void fail(Fault fault)
{
  t.state = State::Fault;
  t.fault = fault;
  armed = false;
  reset();
}
Command publish(Command c = {})
{
  if (t.state == State::Disabled || t.state == State::Fault) {
    c = {};
    t.torque_yaw = t.torque_arm = t.torque_pitch = 0;
    t.speed_command_yaw = t.speed_command_pitch = 0;
    t.pid_yaw = t.ff_yaw = t.pid_pitch = t.ff_pitch = 0;
    t.accel_yaw = t.accel_pitch = 0;
    t.saturation_mask = 0;
  }
  debug_tracking_state = static_cast<int>(t.state);
  debug_tracking_fault = static_cast<int>(t.fault);
  debug_tracking_calibrated = t.calibrated;

  debug_tracking_target_pitch = t.target_pitch;
  debug_tracking_actual_pitch = t.actual_pitch;
  debug_tracking_error_pitch = t.error_pitch;

  debug_tracking_arm_error = t.arm_error;
  debug_tracking_yaw = t.actual_yaw;

  debug_tracking_phase_ms = t.phase_ms;

  debug_tracking_target_yaw = t.target_yaw;
  debug_tracking_speed_yaw = t.speed_yaw;
  debug_tracking_speed_pitch = t.speed_pitch;

  debug_tracking_torque_yaw = t.torque_yaw;
  debug_tracking_speed_command_yaw = t.speed_command_yaw;
  debug_tracking_actual_speed_yaw = t.speed_yaw;

  debug_tracking_error_yaw = t.error_yaw;
  taskENTER_CRITICAL();
  published = t;
  taskEXIT_CRITICAL();

  return c;
}
void enter(State state, uint32_t now)
{
  t.state = state;
  stage_ms = now;
  settling = false;
}
bool settled(bool close, uint32_t now)
{
  if (!close) {
    settling = false;
    return false;
  }
  if (!settling) {
    settling = true;
    settled_ms = now;
  }
  return now - settled_ms >= kSettleDurationMs;
}
void measure(const Feedback & f)
{
  t.actual_yaw = kYawDirection * (f.angle_a - t.yaw_origin);
  t.actual_pitch = f.pitch - t.pitch_origin;
  t.imu_pitch = f.pitch;
  t.arm_error = kArmDirection * (t.arm_hold - f.angle_b);
  t.tip_position = kTipDirection * (f.angle_c - t.tip_origin);
  t.speed_yaw = kYawDirection * f.speed_a;
  t.speed_pitch = f.pitch_rate;
}
}  // namespace
Telemetry read_telemetry()
{
  taskENTER_CRITICAL();
  const auto result = published;
  taskEXIT_CRITICAL();
  return result;
}
Command update(
  bool requested, bool acknowledge_down, bool remote_ok, const Feedback & f, uint32_t now,
  uint32_t gap)
{
  t.stamp_ms = now;
  t.gap_ms = gap;
  measure(f);
  if (!remote_ok) {
    if (t.state != State::Disabled && t.state != State::Fault) fail(Fault::RemoteLost);
    armed = false;
    return publish();
  }
  if (!requested) {
    reset();
    if (acknowledge_down) {
      armed = true;
      t.fault = Fault::None;
    }
    t.state = t.fault == Fault::None ? State::Disabled : State::Fault;
    return publish();
  }
  if (t.state == State::Fault) return publish();
  if (!armed) {
    fail(Fault::NotArmed);
    return publish();
  }
  if (!f.motors_alive) {
    fail(Fault::FeedbackLost);
    return publish();
  }
  if (
    !std::isfinite(f.angle_a) || !std::isfinite(f.angle_b) || !std::isfinite(f.angle_c) ||
    !std::isfinite(f.speed_a) || !std::isfinite(f.speed_b) || !std::isfinite(f.speed_c)) {
    fail(Fault::NonFinite);
    return publish();
  }
  if (gap == 0 || gap > kMaxControlGapMs) {
    fail(Fault::Timing);
    return publish();
  }
  if (t.state == State::Disabled) {
    reset();
    t = {};
    t.stamp_ms = now;
    t.gap_ms = gap;
    t.arm_hold = f.angle_b;
    t.yaw_origin = f.angle_a;
    c_entry = f.angle_c;
    t.tip_origin = f.angle_c;
    square_yaw = square_pitch = 0;
    enter(State::WaitImu, now);
    measure(f);
  }
  const bool imu_finite =
    std::isfinite(f.pitch) && std::isfinite(f.pitch_rate) && std::isfinite(f.roll);
  const bool imu_fresh = f.imu_ready && now - f.imu_stamp_ms <= kImuTimeoutMs;
  if (f.imu_ready && !imu_finite) {
    fail(Fault::NonFinite);
    return publish();
  }
  if (t.state != State::WaitImu && !imu_fresh) {
    fail(Fault::ImuLost);
    return publish();
  }
  if (imu_fresh && (std::fabs(f.pitch) > kMaxAbsolutePitch || std::fabs(f.roll) > kMaxRoll)) {
    fail(Fault::AttitudeLimit);
    return publish();
  }
  if (
    !inside(t.actual_yaw, kYawMin, kYawMax) ||
    (!t.calibrated && std::fabs(f.angle_c - c_entry) > kMaxLevelTravel) ||
    (t.calibrated && (!inside(t.tip_position, kTipMin, kTipMax) ||
                      !inside(t.actual_pitch, kPitchMin, kPitchMax)))) {
    fail(Fault::TravelLimit);
    return publish();
  }
  if (std::fabs(t.arm_error) > kMaxArmDrift) {
    if (!arm_drift) {
      arm_drift = true;
      arm_drift_ms = now;
    }
    if (now - arm_drift_ms >= kErrorDurationMs) {
      fail(Fault::ArmMoved);
      return publish();
    }
  }
  else
    arm_drift = false;
  const float dt = gap * .001f;
  // B's target and integral remain unchanged through leveling/alignment/playback transitions.
  const auto bo = arm_control.step(
    {kArmDirection * t.arm_hold, 0, 0}, kArmDirection * f.angle_b, kArmDirection * f.speed_b, dt);
  Reference yr{0, 0, 0}, pr{0, 0, 0};
  if (t.state == State::WaitImu) {
    const bool still = imu_fresh && std::fabs(f.pitch_rate) < kSettleSpeed &&
                       std::fabs(f.speed_a) < kSettleSpeed && std::fabs(f.speed_b) < kSettleSpeed &&
                       std::fabs(f.speed_c) < kSettleSpeed && std::fabs(t.arm_error) < kSettleError;
    if (!still)
      warm = false;
    else if (!warm) {
      warm = true;
      warm_ms = now;
    }
    else if (now - warm_ms >= kImuWarmupMs) {
      level_start = f.pitch;
      tip_control.reset();
      enter(State::Level, now);
    }
    if (t.state == State::WaitImu && now - stage_ms > kWaitImuTimeoutMs) {
      fail(Fault::ImuWaitTimeout);
      return publish();
    }
  }
  if (t.state == State::Level) {
    pr = approach(level_start, 0, (now - stage_ms) * .001f, kLevelDurationMs * .001f);
    const bool close =
      now - stage_ms >= kLevelDurationMs && std::fabs(f.pitch) < kLevelTolerance &&
      std::fabs(f.pitch_rate) < kSettleSpeed && std::fabs(f.speed_c) < kSettleSpeed &&
      std::fabs(t.actual_yaw) < kSettleError && std::fabs(f.speed_a) < kSettleSpeed &&
      std::fabs(t.arm_error) < kSettleError && std::fabs(f.speed_b) < kSettleSpeed;
    if (settled(close, now)) {
      // Record the physical horizontal point. Do not reset the IMU attitude estimator.
      t.pitch_origin = f.pitch;
      t.tip_origin = f.angle_c;
      t.yaw_origin = f.angle_a;
      t.arm_at_origin = f.angle_b;
      t.calibrated = true;
      measure(f);
      yaw_control.reset();
      tip_control.reset();
      enter(State::Align, now);
    }
    else if (now - stage_ms > kLevelTimeoutMs) {
      fail(Fault::LevelTimeout);
      return publish();
    }
  }
  if (t.state == State::Align || t.state == State::WaitTrajectory || t.state == State::Running) {
    const auto snap = trajectory_snapshot();
    if (!snap.ready || now - snap.stamp_ms > kReferenceTimeoutMs) {
      fail(Fault::ReferenceStale);
      return publish();
    }
    const auto first = trajectory_sample_at_ms(0);
    if (t.state == State::Align) {
      yr = approach(0, first.yaw, (now - stage_ms) * .001f, kAlignDurationMs * .001f);
      pr = approach(0, first.pitch, (now - stage_ms) * .001f, kAlignDurationMs * .001f);
      const bool close =
        now - stage_ms >= kAlignDurationMs && std::fabs(first.yaw - t.actual_yaw) < kSettleError &&
        std::fabs(first.pitch - t.actual_pitch) < kSettleError &&
        std::fabs(t.speed_yaw) < kSettleSpeed && std::fabs(f.pitch_rate) < kSettleSpeed &&
        std::fabs(t.arm_error) < kSettleError && std::fabs(f.speed_b) < kSettleSpeed;
      if (settled(close, now)) {
        trajectory_set_running(true);
        enter(State::WaitTrajectory, now);
      }
      else if (now - stage_ms > kAlignTimeoutMs) {
        fail(Fault::AlignTimeout);
        return publish();
      }
    }
    if (t.state == State::WaitTrajectory) {
      yr = {first.yaw, 0, 0};
      pr = {first.pitch, 0, 0};
      if (snap.running)
        enter(State::Running, now);
      else if (now - stage_ms > kReferenceTimeoutMs) {
        fail(Fault::ReferenceStale);
        return publish();
      }
    }
    if (t.state == State::Running) {
      if (!snap.running) {
        fail(Fault::ReferenceStale);
        return publish();
      }
      yr = {snap.point.yaw, snap.point.yaw_vel, snap.point.yaw_acc};
      pr = {snap.point.pitch, snap.point.pitch_vel, snap.point.pitch_acc};
      t.phase_ms = snap.phase_ms;
    }
    if (!finite(yr) || !finite(pr)) {
      fail(Fault::NonFinite);
      return publish();
    }
    // Actual C encoder limits AND required C relative travel with B's residual error.
    if (
      !inside(yr.position, kYawMin, kYawMax) || !inside(pr.position, kPitchMin, kPitchMax) ||
      !inside(pr.position + kArmDirection * (t.arm_at_origin - f.angle_b), kTipMin, kTipMax)) {
      fail(Fault::TravelLimit);
      return publish();
    }
    pr.position += t.pitch_origin;  // absolute IMU frame, including gravity compensation
  }
  const auto ao = yaw_control.step(yr, t.actual_yaw, t.speed_yaw, dt);
  AxisOutput co;
  if (t.state == State::WaitImu) {
    co = tip_control.step(
      {0, 0, 0}, kTipDirection * (f.angle_c - c_entry), kTipDirection * f.speed_c, dt,
      kLevelMaxSpeed);
    t.target_pitch = imu_finite ? f.pitch : 0;
    t.actual_pitch = t.target_pitch;
  }
  else {
    co =
      tip_control.step(pr, f.pitch, f.pitch_rate, dt, t.state == State::Level ? kLevelMaxSpeed : 0);
    t.target_pitch = pr.position - t.pitch_origin;
  }
  t.target_yaw = yr.position;
  t.error_yaw = ao.error;
  t.error_pitch = t.target_pitch - t.actual_pitch;
  t.speed_command_yaw = ao.speed_command;
  t.speed_command_pitch = co.speed_command;
  t.pid_yaw = ao.pid_torque;
  t.ff_yaw = ao.feedforward_torque;
  t.pid_pitch = co.pid_torque;
  t.ff_pitch = co.feedforward_torque;
  t.accel_yaw = ao.acceleration_command;
  t.accel_pitch = co.acceleration_command;
  t.torque_yaw = ao.torque;
  t.torque_arm = bo.torque;
  t.torque_pitch = co.torque;
  t.saturation_mask =
    (ao.saturated ? 1U : 0U) | (bo.saturated ? 2U : 0U) | (co.saturated ? 4U : 0U);
  if (t.state == State::Running) {
    if (
      std::fabs(t.error_yaw) > kMaxTrackingError || std::fabs(t.error_pitch) > kMaxTrackingError) {
      if (!large_error) {
        large_error = true;
        error_ms = now;
      }
      if (now - error_ms >= kErrorDurationMs) {
        fail(Fault::TrackingError);
        return publish();
      }
    }
    else
      large_error = false;
    if (t.samples < UINT32_MAX) {
      ++t.samples;
      if (t.saturation_mask) ++t.saturated_samples;
      square_yaw += double(t.error_yaw) * t.error_yaw;
      square_pitch += double(t.error_pitch) * t.error_pitch;
      t.rms_yaw = std::sqrt(square_yaw / t.samples);
      t.rms_pitch = std::sqrt(square_pitch / t.samples);
      t.peak_error_yaw = std::max(t.peak_error_yaw, std::fabs(t.error_yaw));
      t.peak_error_pitch = std::max(t.peak_error_pitch, std::fabs(t.error_pitch));
    }
  }
  return publish({kYawDirection * ao.torque, kArmDirection * bo.torque, kTipDirection * co.torque});
}
}  // namespace tracking
