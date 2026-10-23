#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tracking_mode/tracking_control/tracking_mode.hpp"

extern sp::RM_Motor motor_b;
extern GimbalData motor_b_data;
extern sp::PID motor_b_pid_speed;

sp::Plotter plotter(&huart1, false);

// clang-format off
extern "C" void plot_task(void const * argument)
{
	(void)argument;

	while (1) {
		const auto t = tracking::read_telemetry();
		if (control_mode == ControlMode::TRACKING || t.state == tracking::State::Fault) {
			// 循迹及其故障保留已测试的18通道，故障退入DISABLED后仍可观察原因。
			plotter.plot(
				t.target_yaw, t.actual_yaw, t.target_pitch, t.actual_pitch, t.arm_hold, t.arm_error,
				t.tip_position, t.error_pitch, t.torque_yaw, t.torque_arm, t.torque_pitch,
				t.rms_yaw, t.rms_pitch, static_cast<float>(t.state), static_cast<float>(t.fault),
				static_cast<float>(t.calibrated), static_cast<float>(t.phase_ms),
				static_cast<float>(t.saturation_mask));
			osDelay(tracking_config::kPlotPeriodMs);
			continue;
		}
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
