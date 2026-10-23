#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "../applications/mission/mission_config.hpp"

// Exercise the production helpers without adding test hooks to the firmware.
#include "../applications/control_task.cpp"
#include "../applications/fold_mode/fold_mode.cpp"

namespace
{
struct EndControlTick {};
uint32_t now_ms = 0U;
RemoteTaskData remote_input = {};
bool remote_available = true;
uint8_t last_frame[8] = {};
uint32_t last_tx_id = 0U;
unsigned send_count = 0U;
uint8_t c_frame[8] = {};
tracking::Telemetry test_tracking{};
tracking::Feedback last_tracking_feedback{};
tracking::Command test_tracking_command{};
bool tracking_armed = false;

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
          motor_a_gravity_compensation_torque_nm == 0.0f &&
          motor_c_data.given_torque == 0.0f && c_frame[4] == 0 && c_frame[5] == 0,
        message);
}

void refresh_feedback(bool include_c = true)
{
  uint8_t frame[8] = {0x0F, 0xFF, 0, 0, 0, 0, 25, 0};
  motor_a.read(frame, now_ms);
  motor_b.read(frame, now_ms);
  if (include_c) motor_c.read(frame, now_ms);
  motor_a.angle = 2.0f;
  motor_b.angle = 1.0f;
  motor_c.angle = -0.3f;
}

void prepare()
{
  now_ms = 0U;
  control_mode = ControlMode::DISABLED;
  control_source = ControlSource::MANUAL;
  mission_controller = mission::Controller{};
  mission_status = {};
  automatic_request = stop_latched = false;
  test_tracking = {};
  test_tracking_command = {};
  tracking_armed = true;
  refresh_feedback();
  remote_available = true;
  remote_input = {};
  remote_input.is_alive = true;
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  remote_input.sw_l = REMOTE_SWITCH_MID;
  distance = static_cast<uint16_t>(min_distance_cm + fold_distance_hysteresis_cm + 10U);
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
  yuntai_unfolded_offset_rad = 1.3f;
  yuntai_folded_offset_rad = 0.2f;
  motor_a_gravity_horizontal_angle_rad = NAN;
  check(calibrate_yuntai_from_max_angle(), "valid offsets calibrate successfully");
  fold_home_calibrated = true;
  run_disable_mode();
  send_motor_commands();
  send_count = 0U;
}

void angle_control_tick(float target_angle_a, float target_angle_b)
{
  const unsigned before = send_count;
  run_motor_angle_control(target_angle_a, target_angle_b);
  check(send_count == before, "angle control only calculates commands");
  send_motor_commands();
  check(send_count == before + 2U, "shared sender emits one frame per command ID");
}

void fold_tick(uint16_t distance_cm, uint32_t tick_ms)
{
  const unsigned before = send_count;
  run_yuntai_fold(distance_cm, tick_ms);
  check(send_count == before, "fold control and fault paths never send CAN directly");
  send_motor_commands();
  check(send_count == before + 2U, "fold commands use the shared two-ID sender");
}

void control_tick()
{
  const unsigned before = send_count;
  try {
    control_task(nullptr);
  }
  catch (const EndControlTick &) {
    check(send_count == before + 2U, "each period sends GM6020 and M2006 frames once");
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

extern "C" bool imu_task_get_data(ImuTaskData * data)
{
  *data = {};
  data->euler.pitch_rad = 0.2f;
  data->pitch_rate_rad_s = 0.3f;
  data->stamp_ms = now_ms;
  return true;
}

// Dispatcher tests isolate routing; the actual tracking controller has separate tests.
namespace tracking
{
Telemetry read_telemetry() { return test_tracking; }
Command update(bool requested, bool down, bool online, const Feedback & f, uint32_t, uint32_t)
{
  last_tracking_feedback = f;
  if (!online && test_tracking.state != State::Disabled && test_tracking.state != State::Fault)
    test_tracking.fault = Fault::RemoteLost;
  if (!requested) {
    if (down) {
      tracking_armed = true;
      test_tracking.fault = Fault::None;
    }
    test_tracking.state = test_tracking.fault == Fault::None ? State::Disabled : State::Fault;
    return {};
  }
  if (test_tracking.fault == Fault::None) {
    if (!tracking_armed) test_tracking.fault = Fault::NotArmed;
    else if (!f.motors_alive) test_tracking.fault = Fault::FeedbackLost;
    else if (!std::isfinite(f.angle_a) || !std::isfinite(f.angle_b) ||
             !std::isfinite(f.angle_c) || !std::isfinite(f.speed_a) ||
             !std::isfinite(f.speed_b) || !std::isfinite(f.speed_c))
      test_tracking.fault = Fault::NonFinite;
  }
  test_tracking.state = test_tracking.fault == Fault::None ? State::WaitImu : State::Fault;
  return test_tracking.fault == Fault::None ? test_tracking_command : Command{};
}
}  // namespace tracking

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
  if (tx_id == motor_a.tx_id) {
    std::copy(tx_data, tx_data + CAN_DATA_LEN, last_frame);
    last_tx_id = tx_id;
  }
  else {
    check(tx_id == motor_c.tx_id, "only the M2006 command group is sent separately");
    std::copy(tx_data, tx_data + CAN_DATA_LEN, c_frame);
    check(tx_data[0] == 0 && tx_data[1] == 0 && tx_data[2] == 0 && tx_data[3] == 0 &&
          tx_data[6] == 0 && tx_data[7] == 0, "unused M2006 slots remain zero");
  }
  ++send_count;
}
}  // namespace sp

int main()
{
  check(fold_hold_duration_ms == 3000U, "folded position retains the requested 3 second hold");
  check(near(yuntai_other_limit_offset_rad, 1.9f) && near(yuntai_unfolded_offset_rad, 1.8f) &&
          near(yuntai_folded_offset_rad, 0.0f), "production defaults preserve the current offsets");
  check(std::isnan(yuntai_min_angle_rad) && std::isnan(yuntai_max_angle_rad) &&
          std::isnan(motor_a_gravity_horizontal_angle_rad),
        "absolute limits and horizontal reference still require startup calibration");
  motor_a.angle = 2.0f;
  check(calibrate_yuntai_from_max_angle(), "measured production defaults pass calibration");
  check(near(yuntai_min_angle_rad, 2.0f) && near(yuntai_max_angle_rad, 3.9f) &&
          near(yuntai_folded_angle_rad, 2.0f) && near(yuntai_unfolded_angle_rad, 3.8f),
        "physical endpoint names do not reverse the numerical safety limits");
  check(motor_a.rx_id == 0x206 && motor_b.rx_id == 0x205,
        "folding A uses physical ID 2 and holding B uses physical ID 1");

  prepare();
  fold_home_calibrated = false;
  motor_a_gravity_horizontal_angle_rad = NAN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(!fold_control_initialized, "MID cannot bypass startup DOWN calibration");
  check(control_mode == ControlMode::DISABLED, "missing home calibration selects DISABLED");
  check_zero_commands("MID without calibration keeps both motors disabled");

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD, "calibrated MID selects FOLD");
  check(fold_control_initialized && fold_stage == FoldStage::UNFOLDING &&
          near(motor_a_data.target_angle_set, motor_a.angle + 1.3f) &&
          near(motor_b_data.target_angle_set, motor_b.angle),
        "calibrated MID unfolds A while holding B at its current angle");
  check(raw_slot(1) > 0 && raw_slot(0) == 0,
        "MID sends lifting current to physical ID 2, not the holding motor ID 1");

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
  check(near(yuntai_min_angle_rad, motor_a.angle) &&
          near(yuntai_max_angle_rad, motor_a.angle + 1.5f) &&
          near(yuntai_folded_angle_rad, motor_a.angle + 0.2f) &&
          near(yuntai_unfolded_angle_rad, motor_a.angle + 1.3f),
        "startup down calibration derives both limits and targets from the captured maximum");
  check_zero_commands("startup calibration never applies gravity torque while disabled");
  check(control_mode == ControlMode::DISABLED, "DOWN calibration never selects a motion mode");
  const float captured_reference = motor_a_gravity_horizontal_angle_rad;
  motor_a.angle += 0.4f;
  now_ms = 101U;
  control_tick();
  check(near(motor_a_gravity_horizontal_angle_rad, captured_reference),
        "reference is not recaptured on later down ticks");

  prepare();
  motor_a.angle += 0.2f;
  motor_a.speed = 0.1f;
  angle_control_tick(motor_a.angle + 0.1f, motor_b.angle + 0.2f);
  const float expected_gravity = yuntai::calculate_gravity_compensation(0.2f);
  check(near(motor_a_pid_angle.data.pout, 5.0f * motor_a_pid_angle.data.err[0]) &&
          near(motor_a_pid_angle.data.iout, 2.0f / 1.1f * 0.1f * 0.0005f),
        "A angle PID uses the mapped gains");
  check(near(motor_a_pid_speed.data.pout, 0.3f * motor_a_pid_speed.data.err[0]) &&
          near(motor_a_pid_speed.data.iout,
               0.04f / (1.0f + std::fabs(motor_a_pid_speed.data.err[0])) *
                 motor_a_pid_speed.data.err[0] * 0.0005f), "A speed PID uses the mapped gains");
  check(near(motor_b_pid_angle.data.iout, 1.0f / 1.2f * 0.2f * 0.0005f) &&
          near(motor_b_pid_speed.data.pout, 0.2f * motor_b_pid_speed.data.err[0]) &&
          near(motor_b_pid_speed.data.iout,
               0.08f / (1.0f + std::fabs(motor_b_pid_speed.data.err[0])) *
                 motor_b_pid_speed.data.err[0] * 0.0005f), "B uses its own mapped gains");
  check(near(motor_a_gravity_compensation_torque_nm, expected_gravity) &&
          near(motor_a_data.given_torque, motor_a_pid_speed.out + expected_gravity),
        "A adds actual-angle gravity compensation to the speed PID output");
  check(near(motor_b_data.given_torque, motor_b_pid_speed.out), "B remains PID-only");
  check(last_tx_id == 0x1FE && send_count == 2U, "GM6020 share a frame; M2006 has its own");
  check(raw_slot(1) == static_cast<int16_t>(motor_a_data.given_torque * 16384.0f / 2.223f) &&
          raw_slot(0) == static_cast<int16_t>(motor_b_data.given_torque * 16384.0f / 2.223f),
        "real RM_Motor cmd/write pack folding A into ID 2 and holding B into ID 1");

  prepare();
  motor_a.angle += 0.5f;
  angle_control_tick(motor_a.angle + 4.0f, motor_b.angle + 4.0f);
  check(near(motor_a_data.given_torque, 0.45f), "combined A torque is capped at 0.45 Nm");
  check(near(motor_a_pid_angle.data.err[0], 4.0f), "A preserves non-wrapping mechanical error");
  check(std::fabs(motor_b_data.given_torque) <= 0.3f && motor_b_pid_angle.data.err[0] < 0.0f,
        "B preserves its 0.3 Nm cap and angular error wrapping");

  prepare();
  motor_a.angle += 0.5f;
  yuntai_max_angle_rad = motor_a.angle;
  angle_control_tick(motor_a.angle, motor_b.angle);
  check(raw_slot(1) == 0, "upper limit blocks outward gravity torque even with zero PID error");
  angle_control_tick(motor_a.angle - 0.5f, motor_b.angle);
  check(motor_a_data.given_torque < 0.0f, "upper limit still allows torque back into the range");

  prepare();
  motor_a.angle -= 0.5f;
  yuntai_min_angle_rad = motor_a.angle;
  angle_control_tick(motor_a.angle, motor_b.angle);
  check(raw_slot(1) == 0, "lower limit also blocks outward gravity torque");

  prepare();
  motor_a_gravity_horizontal_angle_rad = NAN;
  angle_control_tick(motor_a.angle + 0.2f, motor_b.angle + 0.2f);
  check_zero_commands("invalid gravity reference disables both commands");

  prepare();
  fold_control_initialized = true;
  fold_stage = FoldStage::FOLDING;
  fold_tick(100U, fold_motion_timeout_ms);
  check(fold_fault == FoldFault::FOLD_TIMEOUT, "fold timeout still latches its original fault");
  check_zero_commands("fold timeout suppresses gravity compensation");

  prepare();
  fold_control_initialized = true;
  motor_a.angle += 0.5f;
  fold_tick(100U, fold_motion_timeout_ms);
  check(fold_fault == FoldFault::UNFOLD_TIMEOUT, "unfold timeout is preserved");
  check_zero_commands("unfold timeout disables both motors");

  prepare();
  motor_a.angle = yuntai_max_angle_rad + 0.1f;
  fold_tick(30U, 0U);
  check(fold_fault == FoldFault::ANGLE_LIMIT, "out-of-range feedback still latches a fault");
  check_zero_commands("angle fault disables both motors");

  prepare();
  fold_control_initialized = true;
  motor_b_hold_angle_rad = motor_b.angle;
  fold_stage = FoldStage::READY;
  motor_a.angle = yuntai_unfolded_angle_rad;
  fold_tick(0xFFFFU, 0U);
  check(fold_stage == FoldStage::READY, "invalid distance cannot trigger folding");
  fold_tick(static_cast<uint16_t>(min_distance_cm + 1U), 0U);
  check(fold_stage == FoldStage::READY, "distance above the configured threshold does not fold");
  fold_tick(min_distance_cm, 0U);
  check(fold_stage == FoldStage::FOLDING && !fold_trigger_armed,
        "the current configured distance threshold triggers folding");
  motor_a.angle = yuntai_folded_angle_rad;
  fold_tick(min_distance_cm, 1U);
  fold_tick(min_distance_cm, 101U);
  check(fold_stage == FoldStage::FOLDED_HOLD, "fold arrival still requires stable feedback");
  const uint32_t unfold_start_ms = 101U + fold_hold_duration_ms;
  fold_tick(100U, unfold_start_ms);
  check(fold_stage == FoldStage::UNFOLDING, "hold duration still starts unfolding");
  motor_a.angle = yuntai_unfolded_angle_rad;
  fold_tick(100U, unfold_start_ms + 1U);
  fold_tick(100U, unfold_start_ms + 101U);
  check(fold_stage == FoldStage::READY, "unfold arrival still returns to READY");
  const auto rearm_distance = static_cast<uint16_t>(min_distance_cm + fold_distance_hysteresis_cm);
  fold_tick(static_cast<uint16_t>(rearm_distance - 1U), unfold_start_ms + 102U);
  check(!fold_trigger_armed, "distance below the configured rearm point remains unarmed");
  fold_tick(rearm_distance, unfold_start_ms + 103U);
  check(fold_trigger_armed, "the configured hysteresis rearms the next fold");

  prepare();
  angle_control_tick(motor_a.angle + 0.2f, motor_b.angle + 0.2f);
  control_tick();
  check_zero_commands("remote down overrides PID and gravity compensation");
  prepare();
  fold_control_initialized = true;
  remote_input.is_alive = false;
  control_tick();
  check(fold_fault == FoldFault::REMOTE_LOST, "remote loss handling is preserved");
  check(control_mode == ControlMode::DISABLED, "remote loss selects DISABLED");
  check_zero_commands("remote loss suppresses gravity compensation");
  prepare();
  fold_control_initialized = true;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  now_ms = 100U;
  control_tick();
  check(fold_fault == FoldFault::FEEDBACK_LOST, "motor feedback timeout handling is preserved");
  check(control_mode == ControlMode::DISABLED, "feedback timeout selects DISABLED");
  check_zero_commands("feedback timeout disables both motors");

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_UP;
  remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::TRACKING && !fold_control_initialized,
        "both switches UP enter TRACKING, not folding");
  check_zero_commands("tracking starts without retaining folding torque");

  // Sentinels verify that staying in TRACKING does not run folding housekeeping.
  fold_control_initialized = true;
  fold_home_settling = true;
  fold_home_settle_start_ms = 123U;
  fold_stage = FoldStage::FOLDED_HOLD;
  fold_stage_start_ms = 456U;
  fold_trigger_armed = false;
  control_tick();
  check(control_mode == ControlMode::TRACKING && fold_control_initialized &&
          fold_home_settling && fold_home_settle_start_ms == 123U &&
          fold_stage == FoldStage::FOLDED_HOLD && fold_stage_start_ms == 456U &&
          !fold_trigger_armed, "steady TRACKING does not update folding state");
  check_zero_commands("steady TRACKING still sends zero current through the shared sender");

  prepare();
  fold_home_calibrated = false;
  motor_a_gravity_horizontal_angle_rad = NAN;
  control_tick();
  check(fold_home_settling, "DOWN starts the calibration settling interval");
  now_ms = 50U;
  refresh_feedback();
  remote_input.sw_r = REMOTE_SWITCH_UP;
  remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::TRACKING && !fold_home_settling &&
          !fold_home_calibrated, "entering TRACKING interrupts pending DOWN calibration");
  now_ms = 200U;
  refresh_feedback();
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  check(!fold_home_calibrated && fold_home_settling && fold_home_settle_start_ms == now_ms,
        "returning to DOWN starts a fresh settling interval instead of reusing elapsed time");
  now_ms = 300U;
  refresh_feedback();
  control_tick();
  check(fold_home_calibrated, "DOWN calibration still completes after an uninterrupted interval");
  check_zero_commands("calibration after a mode switch remains zero torque");

  {
    prepare();
    remote_input.sw_r = REMOTE_SWITCH_UP;
    remote_input.sw_l = REMOTE_SWITCH_MID;
    control_tick();
    check(control_mode == ControlMode::DISABLED && !fold_control_initialized,
          "manual right UP plus left MID selects DISABLED");
    check_zero_commands("unused manual combination remains zero torque");
  }

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(fold_control_initialized, "FOLD starts its internal motion stage");
  check(motor_a_data.given_torque != 0.0f, "FOLD produces a command before switching away");
  remote_input.sw_r = REMOTE_SWITCH_UP;
  remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::TRACKING && !fold_control_initialized,
        "leaving FOLD terminates its internal motion stage");
  check(fold_home_calibrated && fold_fault == FoldFault::NONE,
        "mode switching preserves home calibration without creating a folding fault");
  check_zero_commands("TRACKING cannot reuse the previous folding current");
  check(motor_a_pid_angle.out == 0.0f && motor_a_pid_speed.out == 0.0f &&
          motor_b_pid_angle.out == 0.0f && motor_b_pid_speed.out == 0.0f,
        "mode switching clears all folding PID outputs");
  now_ms = 1000U;
  refresh_feedback();
  motor_b.angle = 1.4f;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD && fold_stage == FoldStage::UNFOLDING &&
          fold_stage_start_ms == now_ms && near(motor_b_hold_angle_rad, motor_b.angle),
        "reentering FOLD starts a fresh motion timer and captures the current hold angle");

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_mode = ControlMode::FOLD;
  fold_control_initialized = true;
  fold_stage = FoldStage::FOLDING;
  now_ms = fold_motion_timeout_ms;
  refresh_feedback();
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::FOLD_TIMEOUT,
        "a folding timeout returns to DISABLED in the same control period");
  check(!fold_control_initialized,
        "a folding timeout also performs exit cleanup in the same control period");
  check_zero_commands("top-level timeout handling sends a zero-current frame");
  remote_input.sw_r = REMOTE_SWITCH_UP;
  remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(fold_fault == FoldFault::FOLD_TIMEOUT,
        "switching to TRACKING cannot clear a latched folding fault");
  check(control_mode == ControlMode::DISABLED, "fold fault also blocks real TRACKING");
  check_zero_commands("TRACKING remains disabled after a folding fault");
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::FOLD_TIMEOUT,
        "returning to MID cannot bypass DOWN fault acknowledgment");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::NONE &&
          fold_home_calibrated, "DOWN acknowledges the fault without losing the home reference");
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD && fold_control_initialized,
        "folding can restart after DOWN acknowledgment");

  prepare();
  fold_control_initialized = true;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  remote_available = false;
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::REMOTE_LOST,
        "missing remote data disables motion and preserves the loss fault");
  check_zero_commands("missing remote data never leaves an old current active");

  prepare();
  fold_control_initialized = true;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  motor_b.speed = NAN;
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::FEEDBACK_LOST,
        "nonfinite feedback prevents selection of a motion mode");
  check_zero_commands("nonfinite feedback sends zero current");

  prepare();
  angle_control_tick(motor_a.angle + 0.2f, motor_b.angle + 0.2f);
  const unsigned before_disable = send_count;
  run_disable_mode();
  check(send_count == before_disable, "shared disable clears commands without sending CAN");
  check(motor_a_pid_angle.out == 0.0f && motor_a_pid_speed.out == 0.0f &&
          motor_b_pid_angle.out == 0.0f && motor_b_pid_speed.out == 0.0f,
        "shared disable still clears every folding PID");
  check(fold_home_calibrated, "shared disable retains the captured home calibration");
  send_motor_commands();
  check_zero_commands("shared disable uses the common zero-torque frame");

  prepare();
  remote_input.sw_r = remote_input.sw_l = REMOTE_SWITCH_UP;
  test_tracking_command = {0.06f, -0.04f, 0.03f};
  control_tick();
  check(control_mode == ControlMode::TRACKING, "tracking request enters its mode branch");
  check(near(last_tracking_feedback.angle_a, motor_b.angle) &&
        near(last_tracking_feedback.angle_b, motor_a.angle) &&
        near(last_tracking_feedback.angle_c, motor_c.angle), "tracking feedback follows IDs 1/2/3");
  check(near(last_tracking_feedback.pitch, 0.2f) && near(last_tracking_feedback.pitch_rate, 0.3f),
        "tracking receives corrected IMU pose and pitch rate");
  check(near(motor_b_data.given_torque, 0.06f) && near(motor_a_data.given_torque, -0.04f) &&
        near(motor_c_data.given_torque, 0.03f), "tracking commands map back to physical IDs");
  check(raw_slot(0) > 0 && raw_slot(1) < 0 &&
        static_cast<int16_t>((c_frame[4] << 8) | c_frame[5]) > 0,
        "tracking yaw/arm/tip occupy the correct separate CAN slots");
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD && test_tracking.state == tracking::State::Disabled &&
        motor_c_data.given_torque == 0, "leaving TRACKING resets it and zeros C before FOLD");

  prepare();
  motor_c.speed = NAN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD, "bad C feedback cannot block original folding");

  prepare();
  remote_input.sw_r = remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  now_ms = 100U;
  refresh_feedback(false);
  control_tick();
  check(control_mode == ControlMode::DISABLED &&
        test_tracking.fault == tracking::Fault::FeedbackLost, "tracking latches loss of C");
  check_zero_commands("tracking loss disables all three motors in the same period");
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::DISABLED, "switching to FOLD cannot bypass tracking fault");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  check(test_tracking.fault == tracking::Fault::None, "DOWN acknowledges tracking faults");

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  gimbal_tracking_request_start();
  control_tick();
  check(control_mode == ControlMode::TRACKING, "program request takes over from MID folding");
  gimbal_tracking_request_stop();
  control_tick();
  check(control_mode == ControlMode::DISABLED && stop_latched, "program stop is latched");
  check_zero_commands("program stop clears all motor commands");
  remote_input.sw_r = remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::DISABLED, "switches cannot bypass latched program stop");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  remote_input.sw_r = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::TRACKING, "DOWN rearms after a program stop");

  prepare();
  tracking_armed = false;
  remote_input.sw_r = remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::DISABLED &&
        test_tracking.fault == tracking::Fault::NotArmed, "startup UP cannot bypass DOWN arming");
  check_zero_commands("unarmed tracking remains zero torque");


  // Automatic selection uses the real dispatcher, original fold controller and common CAN sender.
  prepare();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  control_tick();  // Observe right DOWN before allowing the automatic sequence.
  check(control_source == ControlSource::AUTOMATIC && control_mode == ControlMode::DISABLED &&
          mission_status.armed, "right DOWN overrides automatic selection and arms it");
  check_zero_commands("right DOWN in automatic mode sends zero to all three motors");
  remote_input.sw_r = REMOTE_SWITCH_MID;
  gimbal_tracking_request_start();
  control_tick();
  check(control_mode == ControlMode::FOLD && mission_status.state == mission::State::Fold &&
          mission_status.fold_elapsed_ms == 0U && !automatic_request,
        "left DOWN starts folding and discards legacy tracking requests");
  const uint32_t auto_start_ms = mission_status.started_ms;
  now_ms = 100U;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  remote_input.sw_r = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::FOLD, "right UP with left DOWN remains automatic folding");
  now_ms = 200U;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  control_tick();
  check(fold_stage == FoldStage::READY, "automatic fold stage still uses original arrival logic");
  gimbal_tracking_request_start();
  now_ms = auto_start_ms + (mission_config::kFoldDurationMs - 1U);
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  control_tick();
  check(control_mode == ControlMode::FOLD &&
          mission_status.fold_elapsed_ms == (mission_config::kFoldDurationMs - 1U),
        "program requests cannot skip the automatic fold interval");
  test_tracking_command = {0.06f, -0.04f, 0.03f};
  now_ms = auto_start_ms + mission_config::kFoldDurationMs;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  control_tick();
  check(control_mode == ControlMode::TRACKING && mission_status.state == mission::State::Tracking &&
          !fold_control_initialized, "exactly 7 seconds exits folding and enters tracking");
  check(near(motor_b_data.given_torque, 0.06f) && near(motor_a_data.given_torque, -0.04f) &&
          near(motor_c_data.given_torque, 0.03f), "automatic tracking preserves physical ID mapping");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  check(control_mode == ControlMode::DISABLED && mission_status.state == mission::State::Disabled &&
          mission_status.fold_elapsed_ms == 0U, "right DOWN stops tracking and clears the timer");
  check_zero_commands("right DOWN after automatic tracking sends zero commands");
  now_ms += 1U;
  refresh_feedback();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::FOLD && mission_status.fold_elapsed_ms == 0U,
        "restarting automatic mode always begins with a fresh fold interval");

  prepare();
  control_tick();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  motor_a_pid_speed.data.iout = 0.15f;
  fold_stage = FoldStage::FOLDED_HOLD;
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  control_tick();
  check(control_source == ControlSource::AUTOMATIC && control_mode == ControlMode::FOLD &&
          fold_stage == FoldStage::UNFOLDING && mission_status.fold_elapsed_ms == 0U,
        "manual FOLD to automatic FOLD restarts the sequence despite the same control mode");
  remote_input.sw_l = REMOTE_SWITCH_MID;
  control_tick();
  check(control_source == ControlSource::MANUAL && control_mode == ControlMode::FOLD &&
          mission_status.state == mission::State::Disabled, "leaving automatic restores manual FOLD");
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  control_tick();
  check(control_mode == ControlMode::FOLD && mission_status.fault == mission::Fault::None &&
          mission_status.fold_elapsed_ms == 0U,
        "normal automatic reentry starts a new fold interval without requiring fault acknowledgment");

  prepare();
  fold_home_calibrated = false;
  motor_a_gravity_horizontal_angle_rad = NAN;
  control_tick();
  now_ms = 50U;
  refresh_feedback();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  check(control_mode == ControlMode::DISABLED && mission_status.state == mission::State::WaitReady,
        "automatic selection cannot skip unfinished home calibration");
  now_ms = 20050U;
  refresh_feedback();
  control_tick();
  check(mission_status.state == mission::State::WaitReady && mission_status.fold_elapsed_ms == 0U,
        "waiting without home calibration never advances into tracking");
  check_zero_commands("automatic readiness wait keeps zero torque");

  prepare();
  control_tick();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  remote_input.is_alive = false;
  control_tick();
  check(control_mode == ControlMode::DISABLED && mission_status.fault == mission::Fault::RemoteLost,
        "remote loss terminates automatic operation");
  remote_input.is_alive = true;
  remote_input.sw_r = remote_input.sw_l = REMOTE_SWITCH_UP;
  control_tick();
  check(control_mode == ControlMode::DISABLED && mission_status.fault == mission::Fault::RemoteLost,
        "manual tracking selection cannot bypass an automatic loss fault");
  check_zero_commands("automatic remote fault stays zero after manual selection");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  control_tick();
  check(mission_status.fault == mission::Fault::None, "right DOWN acknowledges the automatic fault");

  prepare();
  control_tick();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  now_ms = fold_motion_timeout_ms + 1U;
  refresh_feedback();
  control_tick();
  check(control_mode == ControlMode::DISABLED && fold_fault == FoldFault::UNFOLD_TIMEOUT &&
          mission_status.fault == mission::Fault::ControlFault,
        "automatic folding faults latch and disable in the same control period");
  check_zero_commands("automatic fold timeout clears all commands");

  prepare();
  control_tick();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  control_tick();
  gimbal_tracking_request_stop();
  control_tick();
  check(control_mode == ControlMode::DISABLED && mission_status.fault == mission::Fault::ProgramStop,
        "program stop also aborts automatic folding");
  check_zero_commands("program stop in automatic mode stays zero torque");

  prepare();
  control_tick();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_UP;
  control_tick();
  now_ms = 100U;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  control_tick();
  now_ms = 200U;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  control_tick();
  now_ms = mission_config::kFoldDurationMs;
  refresh_feedback();
  motor_a.angle = yuntai_unfolded_angle_rad;
  motor_c.speed = NAN;
  control_tick();
  check(control_mode == ControlMode::DISABLED && test_tracking.fault == tracking::Fault::NonFinite &&
          mission_status.fault == mission::Fault::ControlFault,
        "bad C feedback at the timed handover cannot retain fold torque or start tracking");
  check_zero_commands("failed automatic tracking entry zeroes both CAN command groups");

  std::cout << "PASS: mapped PID gains, startup calibration, gravity sum, torque cap, CAN slots, "
               "travel limits, disable priorities, fault paths, folding sequence and mode transitions\n";
}
