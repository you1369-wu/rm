#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

typedef struct
{
	float target_angle_set;	 // 绝对角度的目标值，rad
	float target_speed_set;	 // 绝对速度的目标值，rad/s
	float given_torque;		 // 电机给定的力矩，Nm
} GimbalData;

// 公共失能入口只清理控制输出，CAN 命令由 control_task 在周期末统一发送。
void run_disable_mode();

#endif	// CONTROL_TASK_HPP
