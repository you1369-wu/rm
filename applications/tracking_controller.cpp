#include "tracking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace tracking
{
bool finite(Reference r)
{
  return std::isfinite(r.position) && std::isfinite(r.velocity) && std::isfinite(r.acceleration);
}

AxisOutput AxisController::step(Reference r, float position, float velocity, float dt)
{
  if (
    !finite(r) || !std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(dt) ||
    dt <= 0.0f || dt > 0.005f) {
    reset();
    return {};
  }
  AxisOutput out;
  // 有限行程轴使用连续坐标差，不能把跨越机械限位的运动折算成“最短角”。
  out.error = r.position - position;
  const float speed_raw = gains_.angle_kp * out.error + gains_.velocity_ff * r.velocity;
  out.speed_command = std::clamp(speed_raw, -gains_.max_speed, gains_.max_speed);
  const float speed_error = out.speed_command - velocity;
  const float feedforward =
    gains_.acceleration_ff * r.acceleration + gains_.gravity_ff * std::cos(position);
  const float base = gains_.speed_kp * speed_error + feedforward;
  const float candidate = std::clamp(
    integral_ + gains_.speed_ki * speed_error * dt, -gains_.max_integral, gains_.max_integral);
  const float candidate_output = base + candidate;
  // 条件积分：饱和时拒绝加深饱和的增量，允许积分向解除饱和方向回退。
  const float increment = candidate - integral_;
  if (!((candidate_output > gains_.max_torque && increment > 0.0f) ||
        (candidate_output < -gains_.max_torque && increment < 0.0f))) {
    integral_ = candidate;
  }
  const float raw = base + integral_;
  out.torque = std::clamp(raw, -gains_.max_torque, gains_.max_torque);
  out.saturated = speed_raw != out.speed_command || raw != out.torque;
  return out;
}

Reference approach(float start, float end, float elapsed_s, float duration_s)
{
  const float s = std::clamp(elapsed_s / duration_s, 0.0f, 1.0f);
  const float s2 = s * s, s3 = s2 * s;
  const float delta = end - start;
  // 五次多项式，起终点速度、加速度均为零。
  return {
    start + delta * (10.0f * s3 - 15.0f * s3 * s + 6.0f * s3 * s2),
    delta * (30.0f * s2 - 60.0f * s3 + 30.0f * s2 * s2) / duration_s,
    delta * (60.0f * s - 180.0f * s2 + 120.0f * s3) / (duration_s * duration_s)};
}
}  // namespace tracking
