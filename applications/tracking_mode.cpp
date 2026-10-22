#include "tracking_mode.hpp"

#include <algorithm>
#include <cmath>

#include "FreeRTOS.h"
#include "task.h"
#include "trajectory_runtime.hpp"

namespace tracking
{
namespace
{
using namespace tracking_config;
AxisController yaw_controller(kYaw), pitch_controller(kTip), arm_controller(kArm);
bool calibrated = false, calibrating = false, captured_once = false;
float stable_a = 0, stable_b = 0, stable_c = 0;
uint32_t calibration_ms = 0;
float tip_origin = 0, arm_start = 0;
Telemetry current{}, published{};
bool armed = false;
float yaw_origin = 0, pitch_origin = 0, yaw_start = 0, pitch_start = 0;
uint32_t entry_ms = 0, settled_ms = 0, large_error_ms = 0;
bool settling = false, large_error = false;
double error_square_yaw = 0, error_square_pitch = 0;

bool inside(float value, float low, float high) { return value >= low && value <= high; }
void clear_controllers()
{
  yaw_controller.reset();
  pitch_controller.reset();
  arm_controller.reset();
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
    current.torque_yaw = current.torque_pitch = current.torque_arm = 0;
    current.speed_command_yaw = current.speed_command_pitch = 0;
    current.saturation_mask = 0;
    command = {};
  }
  current.calibrated = calibrated;
  taskENTER_CRITICAL();
  published = current;
  taskEXIT_CRITICAL();
  return command;
}
}  // namespace

Telemetry read_telemetry()
{
  taskENTER_CRITICAL();
  const Telemetry result = published;
  taskEXIT_CRITICAL();
  return result;
}

Command update(
  bool requested, bool acknowledge_down, bool remote_ok, const Feedback & f, uint32_t now_ms,
  uint32_t gap_ms)
{
  const float yaw_encoder = f.angle_a;
  const float pitch_encoder = f.angle_b;
  const bool valid = std::isfinite(f.angle_a) && std::isfinite(f.angle_b) &&
                     std::isfinite(f.speed_a) && std::isfinite(f.speed_b) &&
                     std::isfinite(f.angle_c) && std::isfinite(f.speed_c);
  if (!f.alive || !valid) {
    calibrated = false;
    calibrating = false;
  }
  // 首次上电不依赖遥控器；反馈丢失后的重标定必须先退出运行并回右下档。
  const bool can_capture = !captured_once || (!requested && remote_ok && acknowledge_down);
  const bool stationary = f.alive && valid && std::fabs(f.speed_a) < kCalibrationSpeed &&
                          std::fabs(f.speed_b) < kCalibrationSpeed &&
                          std::fabs(f.speed_c) < kCalibrationSpeed && gap_ms > 0 &&
                          gap_ms <= kMaxControlGapMs;
  if (!calibrated && can_capture && stationary) {
    if (
      !calibrating || std::fabs(f.angle_a - stable_a) > kCalibrationMotion ||
      std::fabs(f.angle_b - stable_b) > kCalibrationMotion ||
      std::fabs(f.angle_c - stable_c) > kCalibrationMotion) {
      calibrating = true;
      calibration_ms = now_ms;
      stable_a = f.angle_a;
      stable_b = f.angle_b;
      stable_c = f.angle_c;
    }
    if (now_ms - calibration_ms >= kCalibrationDurationMs) {
      yaw_origin = f.angle_a;
      pitch_origin = f.angle_b;
      tip_origin = f.angle_c;
      calibrated = true;
      captured_once = true;
      calibrating = false;
    }
  }
  else
    calibrating = false;
  current.gap_ms = gap_ms;
  // 即使遥控器未连接，也发布角度反馈，便于排查通道不更新。
  current.actual_yaw = valid ? kYawDirection * (yaw_encoder - yaw_origin) : 0;
  current.actual_arm = valid ? kPitchDirection * (pitch_encoder - pitch_origin) : 0;
  current.actual_tip = valid ? kTipDirection * (f.angle_c - tip_origin) : 0;
  current.actual_pitch = current.actual_arm + current.actual_tip;
  current.actual_speed_yaw = kYawDirection * f.speed_a;
  current.actual_speed_pitch = kPitchDirection * f.speed_b + kTipDirection * f.speed_c;

  if (!remote_ok) {
    if (
      current.state == State::Align || current.state == State::Wait ||
      current.state == State::Running)
      fail(Fault::RemoteLost);
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
    return publish();
  }
  if (current.state == State::Fault) return publish();
  if (!armed) {
    fail(Fault::NotArmed);
    return publish();
  }
  if (!f.alive) {
    fail(Fault::FeedbackLost);
    return publish();
  }
  if (!valid) {
    fail(Fault::NonFinite);
    return publish();
  }
  if (!calibrated) {
    fail(Fault::NotCalibrated);
    return publish();
  }
  if (gap_ms == 0 || gap_ms > kMaxControlGapMs) {
    fail(Fault::Timing);
    return publish();
  }
  const auto snapshot = trajectory_snapshot();
  if (!snapshot.ready || now_ms - snapshot.stamp_ms > kReferenceTimeoutMs) {
    fail(Fault::ReferenceStale);
    return publish();
  }
  if (current.state == State::Disabled) {
    clear_controllers();
    current = {};
    current.state = State::Align;
    current.gap_ms = gap_ms;
    entry_ms = now_ms;
    error_square_yaw = error_square_pitch = 0;
    // 进入/退出运行不能重设上电零点。
    yaw_start = kYawDirection * (yaw_encoder - yaw_origin);
    arm_start = kPitchDirection * (pitch_encoder - pitch_origin);
    pitch_start = arm_start + kTipDirection * (f.angle_c - tip_origin);
  }
  current.actual_yaw = kYawDirection * (yaw_encoder - yaw_origin);
  current.actual_arm = kPitchDirection * (pitch_encoder - pitch_origin);
  current.actual_tip = kTipDirection * (f.angle_c - tip_origin);
  current.actual_pitch = current.actual_arm + current.actual_tip;
  current.actual_speed_yaw = kYawDirection * f.speed_a;
  current.actual_speed_pitch = kPitchDirection * f.speed_b;
  const float arm_speed = kPitchDirection * f.speed_b;
  current.actual_speed_pitch = arm_speed + kTipDirection * f.speed_c;
  if (
    !inside(current.actual_arm, kArmMin, kArmMax) ||
    !inside(current.actual_tip, kTipMin, kTipMax) ||
    !inside(current.actual_yaw, kYawMin, kYawMax) ||
    !inside(current.actual_pitch, kPitchMin, kPitchMax)) {
    fail(Fault::TravelLimit);
    return publish();
  }
  const auto initial = trajectory_sample_at_ms(0);
  Reference yaw_reference{initial.yaw, 0, 0}, pitch_reference{initial.pitch, 0, 0};
  Reference arm_reference{kArmHold, 0, 0};
  if (current.state == State::Align) {
    const float elapsed = (now_ms - entry_ms) * 0.001f;
    arm_reference = approach(arm_start, kArmHold, elapsed, kAlignDurationMs * 0.001f);
    yaw_reference = approach(yaw_start, initial.yaw, elapsed, kAlignDurationMs * 0.001f);
    pitch_reference = approach(pitch_start, initial.pitch, elapsed, kAlignDurationMs * 0.001f);
    if (now_ms - entry_ms >= kAlignDurationMs) {
      const bool close = std::fabs(kArmHold - current.actual_arm) < kSettleError &&
                         std::fabs(arm_speed) < kSettleSpeed &&
                         std::fabs(initial.yaw - current.actual_yaw) < kSettleError &&
                         std::fabs(initial.pitch - current.actual_pitch) < kSettleError &&
                         std::fabs(current.actual_speed_yaw) < kSettleSpeed &&
                         std::fabs(current.actual_speed_pitch) < kSettleSpeed;
      if (!close)
        settling = false;
      else if (!settling) {
        settling = true;
        settled_ms = now_ms;
      }
      else if (now_ms - settled_ms >= kSettleDurationMs) {
        trajectory_set_running(true);
        current.state = State::Wait;
      }
    }
  }
  if (current.state == State::Wait && snapshot.running) {
    yaw_controller.reset();
    pitch_controller.reset();
    arm_controller.reset();
    current.state = State::Running;
  }
  if (current.state == State::Running) {
    if (!snapshot.running) {
      fail(Fault::ReferenceStale);
      return publish();
    }
    const auto & point = snapshot.point;
    yaw_reference = {point.yaw, point.yaw_vel, point.yaw_acc};
    pitch_reference = {point.pitch, point.pitch_vel, point.pitch_acc};
    current.phase_ms = snapshot.phase_ms;
  }
  else if (now_ms - entry_ms > kAlignTimeoutMs) {
    fail(Fault::AlignTimeout);
    return publish();
  }
  if (!finite(yaw_reference) || !finite(pitch_reference)) {
    fail(Fault::NonFinite);
    return publish();
  }
  if (
    !inside(yaw_reference.position, kYawMin, kYawMax) ||
    !inside(pitch_reference.position, kPitchMin, kPitchMax)) {
    fail(Fault::TravelLimit);
    return publish();
  }
  const float dt = gap_ms * 0.001f;
  const auto yo =
    yaw_controller.step(yaw_reference, current.actual_yaw, current.actual_speed_yaw, dt);
  // 末端俯仰 = 大臂关节角 + 顶部关节角 + 安装偏置。
  // 不对实测速度求导：顶部加速度前馈默认关闭。
  Reference tip_reference{
    pitch_reference.position - current.actual_arm, pitch_reference.velocity - arm_speed, 0};
  if (!inside(tip_reference.position, kTipMin, kTipMax)) {
    fail(Fault::TravelLimit);
    return publish();
  }
  const auto po =
    pitch_controller.step(tip_reference, current.actual_tip, kTipDirection * f.speed_c, dt);
  const auto ao = arm_controller.step(arm_reference, current.actual_arm, arm_speed, dt);
  current.target_arm = arm_reference.position;
  current.target_tip = tip_reference.position;
  current.torque_arm = ao.torque;
  current.target_yaw = yaw_reference.position;
  current.target_pitch = pitch_reference.position;
  current.error_yaw = yo.error;
  current.error_pitch = po.error;
  current.speed_command_yaw = yo.speed_command;
  current.speed_command_pitch = po.speed_command;
  current.torque_yaw = yo.torque;
  current.torque_pitch = po.torque;
  current.saturation_mask =
    (yo.saturated ? 1U : 0U) | (po.saturated ? 2U : 0U) | (ao.saturated ? 4U : 0U);
  if (current.state == State::Running) {
    const bool too_large = std::fabs(yo.error) > kMaxTrackingError ||
                           std::fabs(po.error) > kMaxTrackingError ||
                           std::fabs(ao.error) > kMaxTrackingError;
    if (!too_large)
      large_error = false;
    else if (!large_error) {
      large_error = true;
      large_error_ms = now_ms;
    }
    else if (now_ms - large_error_ms >= kErrorDurationMs) {
      fail(Fault::TrackingError);
      return publish();
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
  return publish({yaw_torque, kPitchDirection * ao.torque, kTipDirection * po.torque});
}
}  // namespace tracking
