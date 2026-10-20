#pragma once
#include "tracking_controller.hpp"
#include <cstdint>

namespace tracking
{
enum class State : uint32_t { Disabled, Align, Wait, Running, Fault };
enum class Fault : uint32_t
{
  None, NotArmed, FeedbackLost, ReferenceStale, NonFinite,
  TravelLimit, Timing, AlignTimeout, TrackingError, RemoteLost
};
struct Feedback
{
  float angle_a, speed_a, angle_b, speed_b;
  bool alive;
};
struct Telemetry
{
  float target_yaw = 0, actual_yaw = 0, target_pitch = 0, actual_pitch = 0;
  float error_yaw = 0, error_pitch = 0;
  float speed_command_yaw = 0, actual_speed_yaw = 0;
  float speed_command_pitch = 0, actual_speed_pitch = 0;
  float torque_yaw = 0, torque_pitch = 0; // 轨迹坐标系，N*m
  float rms_yaw = 0, rms_pitch = 0;
  float peak_error_yaw = 0, peak_error_pitch = 0;
  uint32_t samples = 0, saturated_samples = 0, phase_ms = 0, gap_ms = 0;
  uint32_t saturation_mask = 0;
  State state = State::Disabled;
  Fault fault = Fault::None;
};
struct Command { float torque_a = 0, torque_b = 0; };
// 只由 control_task 调用；本模块不发送 CAN。
Command update(bool requested, bool acknowledge_down, bool remote_ok, const Feedback & feedback,
               uint32_t now_ms, uint32_t gap_ms);
Telemetry read_telemetry();
} // namespace tracking
