#pragma once

#include <cstdint>

namespace tracking_config
{
// 固定映射：A=底部yaw，B=大臂pitch，C=顶部M2006。
// ID 暂定，烧录前改成实物值；GM6020 使用电流模式。
constexpr uint8_t kYawId = 1, kArmId = 2, kTipId = 1;
constexpr float kTipDirection = 1.0f;
constexpr float kArmHold = 0.0f;
constexpr float kArmMin = -0.30f, kArmMax = 0.30f;
constexpr float kTipMin = -0.90f, kTipMax = 0.90f;
constexpr float kMountPitchOffset = 0.0f;
constexpr float kCalibrationPitch = 0.0f;  // 手动标定时平台水平
constexpr uint32_t kCalibrationDurationMs = 1000;
static_assert(
  kYawId >= 1 && kYawId <= 7 && kArmId >= 1 && kArmId <= 7 && kTipId >= 1 && kTipId <= 8 &&
    kYawId != kArmId,
  "Invalid motor IDs");
static_assert(
  0x204 + kYawId != 0x200 + kTipId && 0x204 + kArmId != 0x200 + kTipId,
  "CAN feedback ID collision");
constexpr float kMotorAZero = 2.86164117f;  // 沿用原工程标定值，rad
constexpr float kMotorBZero = 2.10692263f;
constexpr float kYawDirection = 1.0f;  // 只能为 +1 或 -1
constexpr float kPitchDirection = 1.0f;
constexpr float kYawMin = -0.45f;  // 轨迹坐标系下的软件行程，须按机械结构修改
constexpr float kYawMax = 0.45f;
constexpr float kPitchMin = -0.60f;
constexpr float kPitchMax = 0.60f;

struct AxisGains
{
  float angle_kp;  // 角度 P 环，输出 rad/s
  float speed_kp;  // 速度 PI 环，输出 N*m
  float speed_ki;
  float velocity_ff;      // 速度前馈，无量纲
  float acceleration_ff;  // 惯量前馈，N*m/(rad/s^2)
  float gravity_ff;       // 水平为零时：N*m*cos(q)，符号由实机确认
  float max_speed;
  float max_torque;
  float max_integral;
};
// 起调参数，非实机最优值。沿用原工程速度 Kp=0.035、力矩上限0.3。
// 表格峰值速度约7.15/8.31 rad/s，原工程5 rad/s上限不足。
constexpr AxisGains kYaw{5.0f, 0.035f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.30f, 0.10f};
constexpr AxisGains kArm{5.0f, 0.035f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.30f, 0.10f};
constexpr AxisGains kTip{5.0f, 0.035f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.20f, 0.08f};
static_assert(kTipDirection * kTipDirection == 1.0f, "Invalid tip direction");
static_assert(kArmHold >= kArmMin && kArmHold <= kArmMax, "Arm hold outside limits");
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
constexpr uint32_t kPlotPeriodMs = 10;  // 18通道=75字节，100Hz时约7500B/s
static_assert(
  kYawDirection * kYawDirection == 1.0f && kPitchDirection * kPitchDirection == 1.0f,
  "Direction must be +1 or -1");
static_assert(kYawMin < kYawMax && kPitchMin < kPitchMax, "Invalid travel limits");
}  // namespace tracking_config