#include "mission.hpp"

#include "mission_config.hpp"

namespace mission
{
void Controller::abort(Fault fault)
{
  if (fault == Fault::None) return;
  if (status_.fault == Fault::None) status_.fault = fault;
  status_.state = State::Fault;
  status_.armed = false;
}

Request Controller::update(const Input & input)
{
  // 右下优先于模式选择、故障和计时；保留外部原有的标定/故障确认流程。
  if (input.remote_online && input.disable_requested) {
    status_ = {};
    status_.armed = true;
    return Request::Disabled;
  }
  if (status_.fault != Fault::None) return Request::Disabled;

  if (!input.remote_online) {
    if (status_.state != State::Disabled) abort(Fault::RemoteLost);
    status_.armed = false;
    return Request::Disabled;
  }
  if (!input.selected) {
    // 正常退出清除本次计时，保留已由右下确认的启动许可。
    if (status_.state != State::Disabled) {
      const bool armed = status_.armed;
      status_ = {};
      status_.armed = armed;
    }
    return Request::Disabled;
  }
  if (input.stop_requested) {
    abort(Fault::ProgramStop);
    return Request::Disabled;
  }
  if (input.control_fault) {
    abort(Fault::ControlFault);
    return Request::Disabled;
  }
  if (status_.state == State::Disabled) {
    if (!status_.armed) {
      abort(Fault::NotArmed);
      return Request::Disabled;
    }
    status_.state = State::WaitReady;
  }
  if (input.gap_ms == 0U || input.gap_ms > mission_config::kMaxControlGapMs) {
    abort(Fault::Timing);
    return Request::Disabled;
  }
  if (status_.state == State::WaitReady) {
    // 标定或反馈未就绪时不输出，也不提前消耗 20 秒计时。
    if (!input.motors_ready || !input.fold_ready) return Request::Disabled;
    status_.started_ms = input.now_ms;
    status_.state = State::Fold;
  }
  if (!input.motors_ready) {
    abort(Fault::FeedbackLost);
    return Request::Disabled;
  }
  if (!input.fold_ready) {
    abort(Fault::ControlFault);
    return Request::Disabled;
  }
  if (status_.state == State::Fold) {
    const uint32_t elapsed_ms = input.now_ms - status_.started_ms;
    status_.fold_elapsed_ms = elapsed_ms < mission_config::kFoldDurationMs
      ? elapsed_ms : mission_config::kFoldDurationMs;
    if (elapsed_ms >= mission_config::kFoldDurationMs) status_.state = State::Tracking;
  }
  return status_.state == State::Tracking ? Request::Tracking : Request::Fold;
}
}  // namespace mission
