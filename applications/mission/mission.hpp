#pragma once

#include <cstdint>

namespace mission
{
enum class State { Disabled, WaitReady, Fold, Tracking, Fault };
enum class Fault { None, NotArmed, RemoteLost, FeedbackLost, ControlFault, ProgramStop, Timing };
enum class Request { Disabled, Fold, Tracking };

// 输入由 control_task 每周期快照生成；不访问电机、遥控器或 CAN。
struct Input
{
  bool selected = false;          // 左下选择自动流程。
  bool remote_online = false;     // 遥控器反馈有效且在线。
  bool disable_requested = false;  // 右下优先失能，并确认故障/准备下一次启动。
  bool motors_ready = false;      // 两台 GM6020 在线，角度和速度均为有限值。
  bool fold_ready = false;        // 折叠参考角已标定，且无折叠故障。
  bool control_fault = false;     // 折叠或循迹模块已有锁存故障。
  bool stop_requested = false;    // 程序停止请求，在自动和手动下均有效。
  uint32_t now_ms = 0U;           // 单调系统毫秒计时，允许 uint32_t 自然回绕。
  uint32_t gap_ms = 1U;           // 相邻两次控制周期的实际间隔。
};

struct Status
{
  State state = State::Disabled;
  Fault fault = Fault::None;
  bool armed = false;             // 已观察到右下，且启动许可未因故障或失联撤销。
  uint32_t started_ms = 0U;       // 本次真正进入折叠模式的时刻。
  uint32_t fold_elapsed_ms = 0U;  // 已运行的折叠模式时间，上限为配置的切换时间。
};

class Controller
{
public:
  // 输入：本周期安全条件/选择/时刻；输出：请求的控制模式，不计算力矩或发送 CAN。
  // 仅由 control_task 调用；无延时、阻塞或额外 FreeRTOS 任务。
  Request update(const Input & input);

  // 输入：本周期执行控制器后发现的故障；输出：锁存首个故障并停止请求运动。
  void abort(Fault fault);

  // 输出：内部只读状态；调用者负责跨任务发布快照。
  const Status & status() const { return status_; }

private:
  Status status_{};
};
}  // namespace mission
