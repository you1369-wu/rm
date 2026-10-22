#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

extern sp::RM_Motor motor_b;
extern GimbalData motor_b_data;
extern sp::PID motor_b_pid_speed;

sp::Plotter plotter(&huart1, false);

// clang-format off
extern "C" void plot_task(void const * argument)
{
	(void)argument;

	while (1) {
		// A折叠电机速度环: 目标/实际速度(rad/s)、PID总输出及P/I/D分项(Nm)。
		// 只在临界区内复制数据，串口发送放在临界区外。
		float sample[6];
		taskENTER_CRITICAL();
		sample[0] = motor_b_data.target_speed_set;
		sample[1] = motor_b.speed;
		sample[2] = motor_b_pid_speed.out;  // 不含重力补偿，不等于最终given_torque。
		sample[3] = motor_b_pid_speed.data.pout;
		sample[4] = motor_b_pid_speed.data.iout;
		sample[5] = motor_b_pid_speed.data.dout;
		taskEXIT_CRITICAL();

		plotter.plot(
	sample[0], sample[1], sample[2], sample[3], sample[4], sample[5]);

		osDelay(5);
	}
}
// clang-format on
