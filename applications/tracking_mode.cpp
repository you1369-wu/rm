#include "tracking_mode.hpp"
#include "trajectory_runtime.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <algorithm>
#include <cmath>

namespace tracking
{
namespace
{
using namespace tracking_config;
AxisController yaw_controller(kYaw), pitch_controller(kPitch);
Telemetry current{}, published{};
bool armed = false;
float yaw_origin = 0, pitch_origin = 0, yaw_start = 0, pitch_start = 0;
uint32_t entry_ms = 0, settled_ms = 0, large_error_ms = 0;
bool settling = false, large_error = false;
double error_square_yaw = 0, error_square_pitch = 0;

float wrap(float value)
{
  return std::remainder(value, 6.283185307179586f);
}
bool inside(float value, float low, float high) { return value >= low && value <= high; }
void clear_controllers()
{
  yaw_controller.reset();
  pitch_controller.reset();
  trajectory_set_running(false);
  settling = false;
  large_error = false;
}
void fail(Fault reason)
{
  current.state = State::Fault;
  current.fault = reason;
  armed = false;
  clear_controllers();
}
Command publish(Command command = {})
{
  if (current.state == State::Disabled || current.state == State::Fault) {
    current.torque_yaw = current.torque_pitch = 0;
    current.speed_command_yaw = current.speed_command_pitch = 0;
    current.saturation_mask = 0;
    command = {};
  }
  taskENTER_CRITICAL();
  published = current;
  taskEXIT_CRITICAL();
  return command;
}
}

Telemetry read_telemetry()
{
  taskENTER_CRITICAL();
  const Telemetry result = published;
  taskEXIT_CRITICAL();
  return result;
}

Command update(bool requested, bool acknowledge_down, bool remote_ok, const Feedback & f,
               uint32_t now_ms, uint32_t gap_ms)
{
  const float yaw_encoder = kYawUsesMotorA ? f.angle_a : f.angle_b;
  const float pitch_encoder = kYawUsesMotorA ? f.angle_b : f.angle_a;
  const float yaw_zero = kYawUsesMotorA ? kMotorAZero : kMotorBZero;
  const float pitch_zero = kYawUsesMotorA ? kMotorBZero : kMotorAZero;
  const bool valid = std::isfinite(f.angle_a) && std::isfinite(f.angle_b) &&
                     std::isfinite(f.speed_a) && std::isfinite(f.speed_b);
  current.gap_ms = gap_ms;
  current.actual_speed_yaw = kYawDirection * (kYawUsesMotorA ? f.speed_a : f.speed_b);
  current.actual_speed_pitch = kPitchDirection * (kYawUsesMotorA ? f.speed_b : f.speed_a);

  if (!remote_ok) {
    if (current.state == State::Align || current.state == State::Wait ||
        current.state == State::Running) fail(Fault::RemoteLost);
    armed = false;
    return publish();
  }
  if (!requested) {
    clear_controllers();
    // 必须先回右下档才能解锁/确认故障；其他档位不清除故障。
    if (acknowledge_down) {
      armed = true;
      current.fault = Fault::None;
    }
    current.state = current.fault == Fault::None ? State::Disabled : State::Fault;
    current.actual_yaw = valid ? kYawDirection * wrap(yaw_encoder - yaw_zero) : 0;
    current.actual_pitch = valid ? kPitchDirection * wrap(pitch_encoder - pitch_zero) : 0;
    return publish();
  }
  if (current.state == State::Fault) return publish();
  if (!armed) { fail(Fault::NotArmed); return publish(); }
  if (!f.alive) { fail(Fault::FeedbackLost); return publish(); }
  if (!valid) { fail(Fault::NonFinite); return publish(); }
  if (gap_ms == 0 || gap_ms > kMaxControlGapMs) { fail(Fault::Timing); return publish(); }
  const auto snapshot = trajectory_snapshot();
  if (!snapshot.ready || now_ms - snapshot.stamp_ms > kReferenceTimeoutMs) {
    fail(Fault::ReferenceStale); return publish();
  }
  if (current.state == State::Disabled) {
    clear_controllers();
    current = {};
    current.state = State::Align;
    current.gap_ms = gap_ms;
    entry_ms = now_ms;
    error_square_yaw = error_square_pitch = 0;
    // 仅进入模式时选择与标定零点等价的最近一圈，此后保留连续角度。
    yaw_origin = yaw_encoder - wrap(yaw_encoder - yaw_zero);
    pitch_origin = pitch_encoder - wrap(pitch_encoder - pitch_zero);
    yaw_start = kYawDirection * (yaw_encoder - yaw_origin);
    pitch_start = kPitchDirection * (pitch_encoder - pitch_origin);
  }
  current.actual_yaw = kYawDirection * (yaw_encoder - yaw_origin);
  current.actual_pitch = kPitchDirection * (pitch_encoder - pitch_origin);
  current.actual_speed_yaw = kYawDirection * (kYawUsesMotorA ? f.speed_a : f.speed_b);
  current.actual_speed_pitch = kPitchDirection * (kYawUsesMotorA ? f.speed_b : f.speed_a);
  if (!inside(current.actual_yaw, kYawMin, kYawMax) ||
      !inside(current.actual_pitch, kPitchMin, kPitchMax)) {
    fail(Fault::TravelLimit); return publish();
  }
  const auto initial = trajectory_sample_at_ms(0);
  Reference yaw_reference{initial.yaw, 0, 0}, pitch_reference{initial.pitch, 0, 0};
  if (current.state == State::Align) {
    const float elapsed = (now_ms - entry_ms) * 0.001f;
    yaw_reference = approach(yaw_start, initial.yaw, elapsed, kAlignDurationMs * 0.001f);
    pitch_reference = approach(pitch_start, initial.pitch, elapsed, kAlignDurationMs * 0.001f);
    if (now_ms - entry_ms >= kAlignDurationMs) {
      const bool close = std::fabs(initial.yaw - current.actual_yaw) < kSettleError &&
        std::fabs(initial.pitch - current.actual_pitch) < kSettleError &&
        std::fabs(current.actual_speed_yaw) < kSettleSpeed &&
        std::fabs(current.actual_speed_pitch) < kSettleSpeed;
      if (!close) settling = false;
      else if (!settling) { settling = true; settled_ms = now_ms; }
      else if (now_ms - settled_ms >= kSettleDurationMs) {
        trajectory_set_running(true);
        current.state = State::Wait;
      }
    }
  }
  if (current.state == State::Wait && snapshot.running) {
    yaw_controller.reset();
    pitch_controller.reset();
    current.state = State::Running;
  }
  if (current.state == State::Running) {
    if (!snapshot.running) { fail(Fault::ReferenceStale); return publish(); }
    const auto & point = snapshot.point;
    yaw_reference = {point.yaw, point.yaw_vel, point.yaw_acc};
    pitch_reference = {point.pitch, point.pitch_vel, point.pitch_acc};
    current.phase_ms = snapshot.phase_ms;
  }
  else if (now_ms - entry_ms > kAlignTimeoutMs) {
    fail(Fault::AlignTimeout); return publish();
  }
  if (!finite(yaw_reference) || !finite(pitch_reference)) {
    fail(Fault::NonFinite); return publish();
  }
  if (!inside(yaw_reference.position, kYawMin, kYawMax) ||
      !inside(pitch_reference.position, kPitchMin, kPitchMax)) {
    fail(Fault::TravelLimit); return publish();
  }
  const float dt = gap_ms * 0.001f;
  const auto yo = yaw_controller.step(yaw_reference, current.actual_yaw, current.actual_speed_yaw, dt);
  const auto po = pitch_controller.step(pitch_reference, current.actual_pitch, current.actual_speed_pitch, dt);
  current.target_yaw = yaw_reference.position;
  current.target_pitch = pitch_reference.position;
  current.error_yaw = yo.error;
  current.error_pitch = po.error;
  current.speed_command_yaw = yo.speed_command;
  current.speed_command_pitch = po.speed_command;
  current.torque_yaw = yo.torque;
  current.torque_pitch = po.torque;
  current.saturation_mask = (yo.saturated ? 1U : 0U) | (po.saturated ? 2U : 0U);
  if (current.state == State::Running) {
    const bool too_large = std::fabs(yo.error) > kMaxTrackingError ||
                           std::fabs(po.error) > kMaxTrackingError;
    if (!too_large) large_error = false;
    else if (!large_error) { large_error = true; large_error_ms = now_ms; }
    else if (now_ms - large_error_ms >= kErrorDurationMs) {
      fail(Fault::TrackingError); return publish();
    }
    // 每次正式播放重置统计；启动靠近阶段不计入跟踪成绩。
    if (current.samples < UINT32_MAX) {
      ++current.samples;
      if (current.saturation_mask) ++current.saturated_samples;
      error_square_yaw += static_cast<double>(yo.error) * yo.error;
      error_square_pitch += static_cast<double>(po.error) * po.error;
      current.rms_yaw = static_cast<float>(std::sqrt(error_square_yaw / current.samples));
      current.rms_pitch = static_cast<float>(std::sqrt(error_square_pitch / current.samples));
      current.peak_error_yaw = std::max(current.peak_error_yaw, std::fabs(yo.error));
      current.peak_error_pitch = std::max(current.peak_error_pitch, std::fabs(po.error));
    }
  }
  const float yaw_torque = kYawDirection * yo.torque;
  const float pitch_torque = kPitchDirection * po.torque;
  return publish(kYawUsesMotorA ? Command{yaw_torque, pitch_torque} :
                                 Command{pitch_torque, yaw_torque});
}
} // namespace tracking
