//负责can的收发和电机控制
//负责云台的折叠
//只在ready状态下判断distance，是否需要折叠
#include "control_task.hpp"

#include <algorithm>
#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"
#include "tools/gravity_compensation.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::CAN can1;
extern sp::RM_Motor motor_a;  //云台折叠机构
extern sp::RM_Motor motor_b;  //云台yaw轴
extern volatile uint16_t distance;

GimbalData motor_a_data;
GimbalData motor_b_data;

// 最大限位与水平姿态相同，下档标定成功后同时保存重力补偿参考角。
float motor_a_gravity_horizontal_angle_rad = NAN;
float motor_a_gravity_compensation_torque_nm = 0.0f;
constexpr float kMotorAMaxGivenTorqueNm = 0.45f;

uint16_t min_distance_cm = 30U;             // 折叠触发阈值，单位 cm。
uint16_t fold_distance_hysteresis_cm = 5U;  // 重新待命阈值 = 触发阈值 + 回差。

// 从上电时人工放置的机械最大端起，沿电机角度坐标测得的有符号偏移，单位 rad。
// 实测机械最小端 = 最大端 + 1.5；两目标各向行程内退 0.2，任一偏移为 NAN 则禁止控制。
float yuntai_other_limit_offset_rad = 1.9f;  // 机械最小限位端相对开机最大端。
float yuntai_unfolded_offset_rad = 1.8f;     // 展开 = 机械最小端 - 0.1 = 最大端 + 1.8。
float yuntai_folded_offset_rad = 0.0f;       // 折叠 = 机械最大端 。

// 每次上电在遥控器下档采集一次参考角后，由偏移量计算本次运行的绝对角度。
float yuntai_unfolded_angle_rad = NAN;
float yuntai_folded_angle_rad = NAN;
float yuntai_min_angle_rad = NAN;         // 反馈数值下限，本次对应机械最大端，单位 rad。
float yuntai_max_angle_rad = NAN;         // 反馈数值上限，本次对应机械最小端，单位 rad。
uint32_t fold_motion_timeout_ms = 5000U;  //折叠或复位超时保护（超过5s没到位则异常退出）
uint32_t fold_hold_duration_ms = 5000U;   //保持折叠状态时间

enum class FoldStage
{
  UNFOLDING,
  READY,
  FOLDING,
  FOLDED_HOLD
};  //折叠阶段
enum class FoldFault
{
  NONE,
  FOLD_TIMEOUT,
  UNFOLD_TIMEOUT,
  ANGLE_LIMIT,
  FEEDBACK_LOST,
  REMOTE_LOST
};  //折叠故障类型

bool fold_control_initialized = false;
bool fold_home_calibrated = false;
bool fold_home_settling = false;
uint32_t fold_home_settle_start_ms = 0U;
bool fold_trigger_armed = true;
FoldStage fold_stage = FoldStage::UNFOLDING;
FoldFault fold_fault = FoldFault::NONE;
uint32_t fold_stage_start_ms = 0U;
uint32_t fold_settle_start_ms = 0U;
bool fold_settling = false;
float motor_b_hold_angle_rad = 0.0f;

constexpr float kFoldAngleToleranceRad = 0.15f;      //到位角度容差
constexpr float kFoldSpeedToleranceRadS = 0.1f;      //到位速度容差
constexpr uint32_t kFoldSettleMs = 100U;             //到位时间
constexpr uint32_t kFoldHomeSettleMs = 100U;         // 下档且静止后才采集开机参考角。
constexpr float kLimitFeedbackToleranceRad = 0.02f;  // 限位反馈容差，避免编码器微小抖动误报。

// 参数顺序: dt, kp, ki, kd, mo, mio, alpha, angular, dynamic。
// A 参数来自 feature/motor_b_pid；保留多圈误差，避免跨越机械限位取最短路径。
sp::PID motor_a_pid_angle(0.001f, 5.0f, 2.0f, 0.0f, 5.0f, 2.5f, 1.0f, false, true);
sp::PID motor_a_pid_speed(
  0.001f, 0.3f, 0.04f, 0.0f, kMotorAMaxGivenTorqueNm, 0.2f, 1.0f, false, true);
// B 参数来自 feature/pid_motor_a；该分支没有重力补偿，仍只使用 PID 保持角度。
sp::PID motor_b_pid_angle(0.001f, 4.5f, 1.0f, 0.0f, 5.0f, 2.5f, 1.0f, true, true);
sp::PID motor_b_pid_speed(0.001f, 0.2f, 0.08f, 0.0f, 0.3f, 0.2f, 1.0f, false, true);

void run_disable_mode()
{
  //下档、失联、折叠超时或限位故障均发送两台电机的零力矩。
  motor_a_gravity_compensation_torque_nm = 0.0f;
  motor_a_data.given_torque = 0.0f;
  motor_b_data.given_torque = 0.0f;
  motor_a_data.target_speed_set = 0.0f;
  motor_b_data.target_speed_set = 0.0f;
  motor_a_pid_angle.clear();
  motor_a_pid_speed.clear();
  motor_b_pid_angle.clear();
  motor_b_pid_speed.clear();
  motor_a.cmd(0.0f);
  motor_b.cmd(0.0f);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  // 软件零力矩依赖 CAN 正常发送，不能替代硬件急停；当前 send() 不反馈发送结果。
  can1.send(motor_a.tx_id);
}

//开机下档标定云台折叠机构的限位角度和目标角度
bool calibrate_yuntai_from_max_angle()
{
  const float home_angle = motor_a.angle;
  const float other_limit = home_angle + yuntai_other_limit_offset_rad;
  const float unfolded = home_angle + yuntai_unfolded_offset_rad;
  const float folded = home_angle + yuntai_folded_offset_rad;
  const float lower_limit = std::min(home_angle, other_limit);
  const float upper_limit = std::max(home_angle, other_limit);

  // 三个偏移必须同一次标定取得；行程小于一圈，且两个目标都在安全行程内。
  if (
    !std::isfinite(home_angle) || !std::isfinite(yuntai_other_limit_offset_rad) ||
    !std::isfinite(yuntai_unfolded_offset_rad) || !std::isfinite(yuntai_folded_offset_rad) ||
    !std::isfinite(other_limit) || !std::isfinite(unfolded) || !std::isfinite(folded) ||
    std::fabs(yuntai_other_limit_offset_rad) <= kFoldAngleToleranceRad ||
    std::fabs(yuntai_other_limit_offset_rad) >= 2.0f * sp::SP_PI || unfolded < lower_limit ||
    unfolded > upper_limit || folded < lower_limit || folded > upper_limit) {
    return false;
  }

  // 机械最大端不一定是编码器数值的上端，按实际反馈方向确定数值上下限。
  yuntai_min_angle_rad = lower_limit;
  yuntai_max_angle_rad = upper_limit;
  yuntai_unfolded_angle_rad = unfolded;
  yuntai_folded_angle_rad = folded;
  motor_a_gravity_horizontal_angle_rad = home_angle;
  return true;
}

void run_motor_angle_control(float target_angle_a, float target_angle_b)
{
  // 两台电机各自运行角度/速度双环PID, 控制量写入同一帧后发送。
  motor_a_data.target_angle_set = target_angle_a;
  motor_a_pid_angle.calc(target_angle_a, motor_a.angle);
  motor_a_data.target_speed_set = motor_a_pid_angle.out;
  motor_a_pid_speed.calc(motor_a_data.target_speed_set, motor_a.speed);

  // A 实际角度相对开机水平参考角的变化量，用于计算抬起方向的重力补偿。
  const float delta_angle_rad = motor_a.angle - motor_a_gravity_horizontal_angle_rad;
  motor_a_gravity_compensation_torque_nm = yuntai::calculate_gravity_compensation(delta_angle_rad);
  const float total_torque_nm = motor_a_pid_speed.out + motor_a_gravity_compensation_torque_nm;

  // 参考角或补偿计算异常时，两台电机均发送零力矩，不把无效值交给 cmd()。
  if (!std::isfinite(total_torque_nm)) {
    run_disable_mode();
    return;
  }

  // 总力矩先限幅，再接受原有机械限位检查；补偿不能绕过限位或失能。
  motor_a_data.given_torque = sp::limit_max(total_torque_nm, kMotorAMaxGivenTorqueNm);

  //最大折叠角度限位
  if (
    !std::isfinite(yuntai_min_angle_rad) || !std::isfinite(yuntai_max_angle_rad) ||
    yuntai_min_angle_rad > yuntai_max_angle_rad ||
    (motor_a.angle <= yuntai_min_angle_rad && motor_a_data.given_torque < 0.0f) ||
    (motor_a.angle >= yuntai_max_angle_rad && motor_a_data.given_torque > 0.0f)) {
    motor_a_data.given_torque = 0.0f;
    motor_a_data.target_speed_set = 0.0f;
    motor_a_pid_angle.clear();
    motor_a_pid_speed.clear();
  }

  motor_b_data.target_angle_set = target_angle_b;
  motor_b_pid_angle.calc(target_angle_b, motor_b.angle);
  motor_b_data.target_speed_set = motor_b_pid_angle.out;
  motor_b_pid_speed.calc(motor_b_data.target_speed_set, motor_b.speed);
  motor_b_data.given_torque = motor_b_pid_speed.out;

  motor_a.cmd(motor_a_data.given_torque);
  motor_b.cmd(motor_b_data.given_torque);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

// 故障后获取故障原因，并立即失能
void latch_fold_fault(FoldFault reason)
{
  fold_fault = reason;
  run_disable_mode();
}

//云台折叠
void run_yuntai_fold(uint16_t distance_cm, uint32_t now_ms)
{
  if (!fold_control_initialized) {
    motor_b_hold_angle_rad = motor_b.angle;  //b保持原来的角度不变
    fold_stage = FoldStage::UNFOLDING;
    fold_stage_start_ms = now_ms;
    fold_settling = false;
    fold_trigger_armed = true;
    fold_control_initialized = true;
  }

  // 超过限位角度时锁存故障并失能。
  if (
    !std::isfinite(yuntai_min_angle_rad) || !std::isfinite(yuntai_max_angle_rad) ||
    yuntai_min_angle_rad > yuntai_max_angle_rad || !std::isfinite(yuntai_unfolded_angle_rad) ||
    !std::isfinite(yuntai_folded_angle_rad) || yuntai_unfolded_angle_rad < yuntai_min_angle_rad ||
    yuntai_unfolded_angle_rad > yuntai_max_angle_rad ||
    yuntai_folded_angle_rad < yuntai_min_angle_rad ||
    yuntai_folded_angle_rad > yuntai_max_angle_rad ||
    motor_a.angle < yuntai_min_angle_rad - kLimitFeedbackToleranceRad ||
    motor_a.angle > yuntai_max_angle_rad + kLimitFeedbackToleranceRad) {
    latch_fold_fault(FoldFault::ANGLE_LIMIT);
    return;
  }

  //检查红外测距仪数据合法性
  const bool distance_valid = distance_cm != 0U && distance_cm != 0xFFFFU;

  if (fold_stage == FoldStage::READY) {
    // 回差
    const uint32_t rearm_distance_cm =
      static_cast<uint32_t>(min_distance_cm) + fold_distance_hysteresis_cm;
    if (distance_valid && distance_cm >= rearm_distance_cm) fold_trigger_armed = true;
    if (fold_trigger_armed && distance_valid && distance_cm <= min_distance_cm) {
      fold_stage = FoldStage::FOLDING;
      fold_stage_start_ms = now_ms;
      fold_settling = false;
      fold_trigger_armed = false;
    }
  }
  // 折叠后保持一段时间，再返回展开角度。
  else if (fold_stage == FoldStage::FOLDED_HOLD) {
    if (now_ms - fold_stage_start_ms >= fold_hold_duration_ms) {
      fold_stage = FoldStage::UNFOLDING;
      fold_stage_start_ms = now_ms;
      fold_settling = false;
    }
  }
  else {
    const bool folding = fold_stage == FoldStage::FOLDING;
    const float target = folding ? yuntai_folded_angle_rad : yuntai_unfolded_angle_rad;

    //角度与速度同时进入容差并持续稳定，才认为到位。
    const bool reached = std::fabs(target - motor_a.angle) <= kFoldAngleToleranceRad &&
                         std::fabs(motor_a.speed) <= kFoldSpeedToleranceRadS;

    if (reached) {
      if (!fold_settling) {
        fold_settle_start_ms = now_ms;
        fold_settling = true;
      }
      if (now_ms - fold_settle_start_ms >= kFoldSettleMs) {
        fold_stage = folding ? FoldStage::FOLDED_HOLD : FoldStage::READY;
        fold_stage_start_ms = now_ms;
        fold_settling = false;
      }
    }
    else {
      fold_settling = false;
    }

    // 运动超时后锁存故障，不再自动尝试另一段运动。
    if (
      fold_stage == (folding ? FoldStage::FOLDING : FoldStage::UNFOLDING) &&
      now_ms - fold_stage_start_ms >= fold_motion_timeout_ms) {
      latch_fold_fault(folding ? FoldFault::FOLD_TIMEOUT : FoldFault::UNFOLD_TIMEOUT);
    }
  }

  if (fold_fault != FoldFault::NONE) {
    return;
  }

  //检查云台折叠或展开是否真正到位
  const float target_a = fold_stage == FoldStage::FOLDING || fold_stage == FoldStage::FOLDED_HOLD
                           ? yuntai_folded_angle_rad
                           : yuntai_unfolded_angle_rad;
  run_motor_angle_control(target_a, motor_b_hold_angle_rad);
}

extern "C" void control_task(void const * argument)
{
  (void)argument;

  osDelay(100);

  // 4.4.7-1: 控制任务统一负责 CAN 和折叠/展开，测距由 laser_task 独立采集。
  can1.config();
  can1.start();
  osDelay(100);

  // 初始化
  motor_a_data.target_angle_set = motor_a.angle;
  motor_b_data.target_angle_set = motor_b.angle;

  while (true) {
    RemoteTaskData remote_data = {};
    const bool remote_ready = remote_task_get_data(&remote_data);
    const uint32_t now_ms = osKernelSysTick();
    const bool motors_ready = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

    // 下档零力矩优先；故障锁存后必须经过下档才能重新允许运动。
    if (remote_ready && remote_data.is_alive && remote_data.sw_r == REMOTE_SWITCH_DOWN) {
      fold_fault = FoldFault::NONE;
      fold_control_initialized = false;
      run_disable_mode();

      // 只在本次上电的首次下档、反馈有效且电机静止时采集机械最大限位角度。
      if (
        !fold_home_calibrated && motors_ready && std::isfinite(motor_a.angle) &&
        std::isfinite(motor_a.speed) && std::fabs(motor_a.speed) <= kFoldSpeedToleranceRadS) {
        if (!fold_home_settling) {
          fold_home_settle_start_ms = now_ms;
          fold_home_settling = true;
        }
        if (now_ms - fold_home_settle_start_ms >= kFoldHomeSettleMs)
          fold_home_calibrated = calibrate_yuntai_from_max_angle();
      }
      else {
        fold_home_settling = false;
      }
    }
    else if (!remote_ready || !remote_data.is_alive) {
      fold_home_settling = false;
      if (fold_control_initialized && fold_fault == FoldFault::NONE)
        fold_fault = FoldFault::REMOTE_LOST;
      fold_control_initialized = false;
      run_disable_mode();
    }
    else if (
      !motors_ready || !std::isfinite(motor_a.angle) || !std::isfinite(motor_a.speed) ||
      !std::isfinite(motor_b.angle) || !std::isfinite(motor_b.speed)) {
      fold_home_settling = false;
      if (fold_control_initialized && fold_fault == FoldFault::NONE)
        fold_fault = FoldFault::FEEDBACK_LOST;
      fold_control_initialized = false;
      run_disable_mode();
    }
    else if (remote_data.sw_r != REMOTE_SWITCH_MID && remote_data.sw_r != REMOTE_SWITCH_UP) {
      fold_home_settling = false;
      fold_control_initialized = false;
      run_disable_mode();
    }
    else if (!fold_home_calibrated) {
      // 未采集参考角、偏移量未填或配置无效时，禁止进入角度 PID。
      fold_home_settling = false;
      run_disable_mode();
    }
    else if (fold_fault != FoldFault::NONE) {
      // 故障保持锁存，直到遥控器进入下档；此期间持续发送零力矩。
      run_disable_mode();
    }
    else {
      // 有效距离触发折叠；到位后保持 5 秒，再返回展开角度。
      run_yuntai_fold(distance, now_ms);
    }

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      // 在CAN接收中断中更新电机反馈, control_task使用最新反馈计算PID
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
