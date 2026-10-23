#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#include "mission/mission.hpp"

typedef struct
{
	float target_angle_set;	 // 绝对角度的目标值，rad
	float target_speed_set;	 // 绝对速度的目标值，rad/s
	float given_torque;		 // 电机给定的力矩，Nm
} GimbalData;

enum class ControlMode
{
  DISABLED,
  FOLD,
  TRACKING
};

extern ControlMode control_mode;

// 控制来源和具体运动模式分开：自动流程仍复用原 FOLD/TRACKING 分支。
enum class ControlSource { MANUAL, AUTOMATIC };
extern ControlSource control_source;
// 只由 control_task 发布；供调试观察自动阶段、故障和 20 秒计时。
extern mission::Status mission_status;

// 公共失能入口只清理控制输出，CAN 命令由 control_task 在周期末统一发送。
void run_disable_mode();

#endif	// CONTROL_TASK_HPP
