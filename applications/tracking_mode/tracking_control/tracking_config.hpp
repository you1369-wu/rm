#pragma once
#include <cstdint>
namespace tracking_config
{
// A=yaw GM6020 ID1, B=big pitch GM6020 ID2, C=small pitch M2006 ID3.
constexpr uint8_t kYawId = 1, kArmId = 2, kTipId = 3;
constexpr float kTipRatio = 36.0f;  // output-shaft rad, rad/s, N*m
constexpr float kYawDirection = -1, kArmDirection = 1, kTipDirection = -1;
// Positive C torque must increase corrected IMU pitch. Verify installation.
constexpr float kImuPitchDirection = 1, kImuPitchMountOffset = 0;
constexpr uint32_t kControlPeriodMs = 1, kMaxControlGapMs = 5;
constexpr uint32_t kImuTimeoutMs = 10, kReferenceTimeoutMs = 10;
constexpr uint32_t kImuWarmupMs = 500, kWaitImuTimeoutMs = 5000;
constexpr uint32_t kLevelDurationMs = 2000, kLevelTimeoutMs = 10000;
constexpr uint32_t kAlignDurationMs = 2000, kAlignTimeoutMs = 100000;
constexpr uint32_t kSettleDurationMs = 300, kErrorDurationMs = 200;
constexpr float kLevelTolerance = 0.06f, kSettleError = 0.03f, kSettleSpeed = 0.08f;
constexpr float kLevelMaxSpeed = 0.5f;
constexpr float kYawMin = -2.0f, kYawMax = 2.0f, kPitchMin = -1.0f, kPitchMax = 1.0f;
constexpr float kTipMin = -2.0f, kTipMax = 2.0f;  // from leveled C encoder origin
constexpr float kMaxLevelTravel = 2.5f;           // C travel from entry, before leveling
constexpr float kMaxAbsolutePitch = 2.0f, kMaxRoll = 3.5f;
constexpr float kMaxArmDrift = 0.10f, kMaxTrackingError = 0.35f;
constexpr uint32_t kPlotPeriodMs = 10;
constexpr unsigned kPlotPage = 0;  // 0=assessment, 1=feedforward diagnostics
struct AxisGains
{
  float angle_kp, speed_kp, speed_ki, velocity_ff;
  float inertia_ff;  // J, kg*m^2, output-axis equivalent inertia
  float damping_ff;  // b, N*m*s/rad
  float gravity_ff;  // N*m*cos(physical angle); identify before enabling
  float max_speed, max_torque, max_integral;
  float acceleration_filter_s, max_acceleration;
};
// Starting values only. Motor IDs cannot determine inertia/damping/gravity.
constexpr AxisGains kYaw{5, 0.15f, 0.12f, 1, 0, 0, 0, 10, 0.30f, 0.10f, 0.010f, 200};
constexpr AxisGains kArm{5, 0.20f, 0.08f, 0, 0, 0, 0, 1, 0.30f, 0.10f, 0.010f, 50};
constexpr AxisGains kTip{5, 0.30f, 0.01f, 1, 0, 0, 0, 10, 0.20f, 0.08f, 0.010f, 200};
static_assert(kYawId == 1 && kArmId == 2 && kTipId == 3, "Check motor mapping");
static_assert(kTipRatio > 0 && kPlotPage <= 1, "Invalid configuration");
static_assert(
  kYawDirection * kYawDirection == 1 && kArmDirection * kArmDirection == 1 &&
    kTipDirection * kTipDirection == 1 && kImuPitchDirection * kImuPitchDirection == 1,
  "Directions must be +1 or -1");
}  // namespace tracking_config
