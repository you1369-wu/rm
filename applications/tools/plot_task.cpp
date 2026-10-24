#include <cstdint>

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
extern volatile uint16_t distance;

sp::Plotter plotter(&huart1, false);

// clang-format off
extern "C" void plot_task(void const * argument)
{
	(void)argument;

	while (1) {
		const auto t = tracking::read_telemetry();
		const auto score = tracking_score::read_score();
		const float distance_cm = static_cast<float>(distance);
		// CH1/2=yaw_ratio/pitch_ratio，CH3/4=yaw目标/编码器反馈角(rad)。
		// CH5/6=小pitch目标/IMU反馈角(rad)，CH7=distance(cm)，65535表示无效。
		plotter.plot(
			score.yaw_ratio, score.pitch_ratio,
			t.target_yaw, t.actual_yaw, t.target_tip, t.tip_position,
			distance_cm);
		osDelay(tracking_config::kPlotPeriodMs);
	}
}
// clang-format on
