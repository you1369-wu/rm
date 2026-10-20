#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

// Exercise the production helpers without adding test hooks to the firmware.
#include "../applications/control_task.cpp"

namespace
{
struct EndControlTick {};
uint32_t now_ms = 0U;
RemoteTaskData remote_input = {};
bool remote_available = true;
uint8_t last_frame[8] = {};
uint32_t last_tx_id = 0U;
unsigned send_count = 0U;

void check(bool result, const char * message)
{
  if (!result) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool near(float a, float b) { return std::fabs(a - b) < 1e-6f; }

int16_t raw_slot(unsigned motor_index)
{
  return static_cast<int16_t>((last_frame[2U * motor_index] << 8U) |
                              last_frame[2U * motor_index + 1U]);
}

void check_zero_commands(const char * message)
{
  check(raw_slot(0) == 0 && raw_slot(1) == 0 &&
          motor_a_data.given_torque == 0.0f && motor_b_data.given_torque == 0.0f &&
          motor_a_gravity_compensation_torque_nm == 0.0f,
        message);
}

void refresh_feedback()
{
  uint8_t frame[8] = {0x0F, 0xFF, 0, 0, 0, 0, 25, 0};
  motor_a.read(frame, now_ms);
  motor_b.read(frame, now_ms);
  motor_a.angle = 2.0f;
  motor_b.angle = 1.0f;
}

void prepare()
{
  now_ms = 0U;
  refresh_feedback();
  remote_available = true;
  remote_input = {};
  remote_input.is_alive = true;
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  fold_control_initialized = false;
  fold_home_calibrated = false;
  fold_home_settling = false;
  fold_home_settle_start_ms = 0U;
  fold_settling = false;
  fold_stage = FoldStage::UNFOLDING;
  fold_stage_start_ms = 0U;
  fold_fault = FoldFault::NONE;
  fold_trigger_armed = true;
  yuntai_other_limit_offset_rad = 1.5f;
  yuntai_unfolded_offset_rad = 0.0f;
  yuntai_folded_offset_rad = 1.3f;
  motor_a_gravity_horizontal_angle_rad = NAN;
  check(calibrate_yuntai_from_max_angle(), "valid offsets calibrate successfully");
  fold_home_calibrated = true;
  run_disable_mode();
  send_count = 0U;
}

void control_tick()
{
  try {
    control_task(nullptr);
  }
  catch (const EndControlTick &) {
    return;
  }
  check(false, "control task must reach its periodic delay");
}
}  // namespace

CAN_HandleTypeDef hcan1;
volatile uint16_t distance = 100U;

extern "C" uint32_t osKernelSysTick() { return now_ms; }
extern "C" int osDelay(uint32_t milliseconds)
{
  if (milliseconds == 1U) throw EndControlTick{};
  return 0;
}
extern "C" uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t) { return 0U; }
extern "C" bool remote_task_get_data(RemoteTaskData * data)
{
  *data = remote_input;
  return remote_available;
}

namespace sp
{
CAN::CAN(CAN_HandleTypeDef * hcan) : hcan_(hcan)
{
  std::fill(tx_data, tx_data + CAN_DATA_LEN, 0U);
}
void CAN::config() {}
void CAN::start() {}
void CAN::recv(uint32_t) {}
void CAN::send(uint32_t tx_id)
{
  std::copy(tx_data, tx_data + CAN_DATA_LEN, last_frame);
  last_tx_id = tx_id;
  ++send_count;
}
}  // namespace sp

int main()
{
  check(std::isnan(yuntai_other_limit_offset_rad) && std::isnan(yuntai_unfolded_offset_rad) &&
          std::isnan(yuntai_folded_offset_rad), "unmeasured mechanical offsets remain unset");
  check(motor_a.rx_id == 0x205 && motor_b.rx_id == 0x206,
        "target branch CAN motor IDs are unchanged");

  prepare();
  fold_home_calibrated = false;
  motor_a_gravity_horizontal_angle_rad = NAN;
  control_tick();
  check(!fold_home_calibrated, "first down tick only starts the settling interval");
  now_ms = 100U;
  refresh_feedback();
  control_tick();
  check(fold_home_calibrated && near(motor_a_gravity_horizontal_angle_rad, motor_a.angle),
        "down switch calibration also captures the horizontal reference after settling");
  check_zero_commands("startup calibration never applies gravity torque while disabled");
  const float captured_reference = motor_a_gravity_horizontal_angle_rad;
  motor_a.angle += 0.4f;
  now_ms = 101U;
  control_tick();
  check(near(motor_a_gravity_horizontal_angle_rad, captured_reference),
        "reference is not recaptured on later down ticks");

  prepare();
  motor_a.angle += 0.2f;
  motor_a.speed = 0.1f;
  run_motor_angle_control(motor_a.angle + 0.1f, motor_b.angle + 0.2f);
  const float expected_gravity = yuntai::calculate_gravity_compensation(0.2f);
  check(near(motor_a_pid_angle.data.pout, 5.0f * motor_a_pid_angle.data.err[0]) &&
          near(motor_a_pid_angle.data.iout, 2.0f / 1.1f * 0.1f * 0.0005f),
        "A angle PID uses the mapped gains");
  check(near(motor_a_pid_speed.data.pout, 0.35f * motor_a_pid_speed.data.err[0]) &&
          near(motor_a_pid_speed.data.iout,
               0.04f / (1.0f + std::fabs(motor_a_pid_speed.data.err[0])) *
                 motor_a_pid_speed.data.err[0] * 0.0005f), "A speed PID uses the mapped gains");
  check(near(motor_b_pid_angle.data.iout, 2.51f / 1.2f * 0.2f * 0.0005f) &&
          near(motor_b_pid_speed.data.pout, 0.1f * motor_b_pid_speed.data.err[0]) &&
          near(motor_b_pid_speed.data.iout,
               0.08f / (1.0f + std::fabs(motor_b_pid_speed.data.err[0])) *
                 motor_b_pid_speed.data.err[0] * 0.0005f), "B uses its own mapped gains");
  check(near(motor_a_gravity_compensation_torque_nm, expected_gravity) &&
          near(motor_a_data.given_torque, motor_a_pid_speed.out + expected_gravity),
        "A adds actual-angle gravity compensation to the speed PID output");
  check(near(motor_b_data.given_torque, motor_b_pid_speed.out), "B remains PID-only");
  check(last_tx_id == 0x1FE && send_count == 1U, "both commands use one current-control frame");
  check(raw_slot(0) == static_cast<int16_t>(motor_a_data.given_torque * 16384.0f / 2.223f) &&
          raw_slot(1) == static_cast<int16_t>(motor_b_data.given_torque * 16384.0f / 2.223f),
        "real RM_Motor cmd/write pack A and B into their original slots");

  prepare();
  motor_a.angle += 0.5f;
  run_motor_angle_control(motor_a.angle + 4.0f, motor_b.angle + 4.0f);
  check(near(motor_a_data.given_torque, 0.45f), "combined A torque is capped at 0.45 Nm");
  check(near(motor_a_pid_angle.data.err[0], 4.0f), "A preserves non-wrapping mechanical error");
  check(std::fabs(motor_b_data.given_torque) <= 0.3f && motor_b_pid_angle.data.err[0] < 0.0f,
        "B preserves its 0.3 Nm cap and angular error wrapping");

  prepare();
  motor_a.angle += 0.5f;
  yuntai_max_angle_rad = motor_a.angle;
  run_motor_angle_control(motor_a.angle, motor_b.angle);
  check(raw_slot(0) == 0, "upper limit blocks outward gravity torque even with zero PID error");
  run_motor_angle_control(motor_a.angle - 0.5f, motor_b.angle);
  check(motor_a_data.given_torque < 0.0f, "upper limit still allows torque back into the range");

  prepare();
  motor_a.angle -= 0.5f;
  yuntai_min_angle_rad = motor_a.angle;
  run_motor_angle_control(motor_a.angle, motor_b.angle);
  check(raw_slot(0) == 0, "lower limit also blocks outward gravity torque");

  prepare();
  motor_a_gravity_horizontal_angle_rad = NAN;
  run_motor_angle_control(motor_a.angle + 0.2f, motor_b.angle + 0.2f);
  check_zero_commands("invalid gravity reference disables both commands");

  prepare();
  fold_control_initialized = true;
  fold_stage = FoldStage::FOLDING;
  run_yuntai_fold(100U, fold_motion_timeout_ms);
  check(fold_fault == FoldFault::FOLD_TIMEOUT, "fold timeout still latches its original fault");
  check_zero_commands("fold timeout suppresses gravity compensation");

  prepare();
  fold_control_initialized = true;
  motor_a.angle += 0.5f;
  run_yuntai_fold(100U, fold_motion_timeout_ms);
  check(fold_fault == FoldFault::UNFOLD_TIMEOUT, "unfold timeout is preserved");
  check_zero_commands("unfold timeout disables both motors");

  prepare();
  motor_a.angle = yuntai_max_angle_rad + 0.1f;
  run_yuntai_fold(30U, 0U);
  check(fold_fault == FoldFault::ANGLE_LIMIT, "out-of-range feedback still latches a fault");
  check_zero_commands("angle fault disables both motors");

  prepare();
  fold_control_initialized = true;
  motor_b_hold_angle_rad = motor_b.angle;
  fold_stage = FoldStage::READY;
  run_yuntai_fold(0xFFFFU, 0U);
  check(fold_stage == FoldStage::READY, "invalid distance cannot trigger folding");
  run_yuntai_fold(30U, 0U);
  check(fold_stage == FoldStage::FOLDING && !fold_trigger_armed, "30 cm still triggers folding");
  motor_a.angle = yuntai_folded_angle_rad;
  run_yuntai_fold(30U, 1U);
  run_yuntai_fold(30U, 101U);
  check(fold_stage == FoldStage::FOLDED_HOLD, "fold arrival still requires stable feedback");
  run_yuntai_fold(100U, 101U + fold_hold_duration_ms);
  check(fold_stage == FoldStage::UNFOLDING, "hold duration still starts unfolding");
  motor_a.angle = yuntai_unfolded_angle_rad;
  run_yuntai_fold(100U, 5102U);
  run_yuntai_fold(100U, 5202U);
  check(fold_stage == FoldStage::READY, "unfold arrival still returns to READY");
  run_yuntai_fold(34U, 5203U);
  check(!fold_trigger_armed, "34 cm remains inside the hysteresis band");
  run_yuntai_fold(35U, 5204U);
  check(fold_trigger_armed, "35 cm still rearms the next fold");

  prepare();
  run_motor_angle_control(motor_a.angle + 0.2f, motor_b.angle + 0.2f);
  control_tick();
  check_zero_commands("remote down overrides PID and gravity compensation");
  prepare();
  fold_control_initialized = true;
  remote_input.is_alive = false;
  control_tick();
  check(fold_fault == FoldFault::REMOTE_LOST, "remote loss handling is preserved");
  check_zero_commands("remote loss suppresses gravity compensation");
  prepare();
  fold_control_initialized = true;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  now_ms = 100U;
  control_tick();
  check(fold_fault == FoldFault::FEEDBACK_LOST, "motor feedback timeout handling is preserved");
  check_zero_commands("feedback timeout disables both motors");

  std::cout << "PASS: mapped PID gains, startup calibration, gravity sum, torque cap, CAN slots, "
               "travel limits, disable priorities, fault paths and folding sequence\n";
}
