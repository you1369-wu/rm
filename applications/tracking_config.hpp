#pragma once

#include <cstdint>

namespace tracking_config
{
// 必须按实物确认：默认 A(ID1)=yaw，B(ID2)=pitch；改变此值可交换映射。
constexpr bool kYawUsesMotorA = true;
constexpr float kMotorAZero = 2.86164117f;  // 沿用原工程标定值，rad
constexpr float kMotorBZero = 2.10692263f;
constexpr float kYawDirection = 1.0f;      // 只能为 +1 或 -1
constexpr float kPitchDirection = 1.0f;
constexpr float kYawMin = -0.45f;         // 轨迹坐标系下的软件行程，须按机械结构修改
constexpr float kYawMax = 0.45f;
constexpr float kPitchMin = -0.60f;
constexpr float kPitchMax = 0.60f;

struct AxisGains
{
  float angle_kp;       // 角度 P 环，输出 rad/s
  float speed_kp;       // 速度 PI 环，输出 N*m
  float speed_ki;
  float velocity_ff;    // 速度前馈，无量纲
  float acceleration_ff; // 惯量前馈，N*m/(rad/s^2)
  float gravity_ff;     // 水平为零时：N*m*cos(q)，符号由实机确认
  float max_speed;
  float max_torque;
  float max_integral;
};
// 起调参数，非实机最优值。沿用原工程速度 Kp=0.035、力矩上限0.3。
// 表格峰值速度约7.15/8.31 rad/s，原工程5 rad/s上限不足。
constexpr AxisGains kYaw{5.0f, 0.035f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.30f, 0.10f};
constexpr AxisGains kPitch{5.0f, 0.035f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.30f, 0.10f};
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint32_t kMaxControlGapMs = 5;
constexpr uint32_t kReferenceTimeoutMs = 10;
constexpr uint32_t kAlignDurationMs = 2000;
constexpr uint32_t kAlignTimeoutMs = 6000;
constexpr uint32_t kSettleDurationMs = 200;
constexpr float kSettleError = 0.03f;
constexpr float kSettleSpeed = 0.15f;
constexpr float kMaxTrackingError = 0.35f;
constexpr uint32_t kErrorDurationMs = 200;
constexpr uint32_t kPlotPeriodMs = 10; // 18通道=75字节，100Hz时约7500B/s
static_assert(kYawDirection * kYawDirection == 1.0f &&
              kPitchDirection * kPitchDirection == 1.0f, "Direction must be +1 or -1");
static_assert(kYawMin < kYawMax && kPitchMin < kPitchMax, "Invalid travel limits");
} // namespace tracking_config
