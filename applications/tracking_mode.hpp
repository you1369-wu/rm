#pragma once
#include <cstdint>
#include "tracking_controller.hpp"
namespace tracking {
enum class State:uint32_t { Disabled,WaitImu,Level,Align,WaitTrajectory,Running,Fault };
enum class Fault:uint32_t {
  None,NotArmed,FeedbackLost,ImuLost,NonFinite,TravelLimit,Timing,LevelTimeout,
  AlignTimeout,ReferenceStale,TrackingError,RemoteLost,ArmMoved,ImuWaitTimeout,AttitudeLimit
};
struct Feedback {
  float angle_a=0,speed_a=0,angle_b=0,speed_b=0,angle_c=0,speed_c=0;
  bool motors_alive=false;
  float pitch=0,pitch_rate=0,roll=0; // corrected IMU pitch, Euler pitch rate
  uint32_t imu_stamp_ms=0;
  bool imu_ready=false;
};
struct Telemetry {
  float target_yaw=0,actual_yaw=0,target_pitch=0,actual_pitch=0;
  float error_yaw=0,error_pitch=0,arm_error=0,tip_position=0;
  float torque_yaw=0,torque_arm=0,torque_pitch=0;
  float rms_yaw=0,rms_pitch=0,peak_error_yaw=0,peak_error_pitch=0;
  float speed_command_yaw=0,speed_yaw=0,speed_command_pitch=0,speed_pitch=0;
  float pid_yaw=0,ff_yaw=0,pid_pitch=0,ff_pitch=0,accel_yaw=0,accel_pitch=0;
  float imu_pitch=0,pitch_origin=0,yaw_origin=0,arm_hold=0,arm_at_origin=0,tip_origin=0;
  uint32_t samples=0,saturated_samples=0,phase_ms=0,gap_ms=0,stamp_ms=0,saturation_mask=0;
  bool calibrated=false;
  State state=State::Disabled;
  Fault fault=Fault::None;
};
struct Command { float torque_a=0,torque_b=0,torque_c=0; };
// Called only by control_task. Previous stage must release CAN ownership before requesting.
Command update(bool requested,bool acknowledge_down,bool remote_ok,
               const Feedback &f,uint32_t now_ms,uint32_t gap_ms);
Telemetry read_telemetry();
}
