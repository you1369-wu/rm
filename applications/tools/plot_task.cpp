#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tracking_mode/tracking_control/tracking_mode.hpp"
#include "tracking_mode/tracking_score/tracking_score_task.h"

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
		const auto score = tracking_score::read_score();
		// CH1/2=tip目标/实际pitch(rad，实际值取IMU)，CH3/4=yaw目标/实际角。
		// 保留18通道：CH5=pitch_ratio，CH6=yaw_ratio，CH7=tip编码器相对角。
		plotter.plot(
			t.target_pitch, t.actual_pitch, t.target_yaw, t.actual_yaw,
			score.pitch_ratio, score.yaw_ratio,
			t.tip_position, t.error_pitch, t.torque_yaw, t.torque_arm, t.torque_pitch,
			t.rms_yaw, t.rms_pitch, static_cast<float>(t.state), static_cast<float>(t.fault),
			static_cast<float>(t.calibrated), static_cast<float>(t.phase_ms),
			static_cast<float>(t.saturation_mask));
		osDelay(tracking_config::kPlotPeriodMs);
	}
}
// clang-format on
