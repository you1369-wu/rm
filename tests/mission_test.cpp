#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "mission/mission.hpp"
#include "mission/mission_config.hpp"

namespace
{
void check(bool result, const char * message)
{
  if (!result) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

mission::Input ready_input(uint32_t now_ms = 0U)
{
  mission::Input input{};
  input.selected = true;
  input.remote_online = true;
  input.motors_ready = input.fold_ready = true;
  input.now_ms = now_ms;
  return input;
}

void arm(mission::Controller & controller, mission::Input & input)
{
  input.disable_requested = true;
  check(controller.update(input) == mission::Request::Disabled,
        "right DOWN never requests motion");
  check(controller.status().armed && controller.status().fault == mission::Fault::None,
        "right DOWN acknowledges and arms the next run");
  input.disable_requested = false;
}
}  // namespace

int main()
{
  using namespace mission;
  check(mission_config::kFoldDurationMs == 7000U,
        "automatic folding duration is seven seconds in total, not seven plus three");
  Controller controller;
  Input input = ready_input();
  check(controller.update(input) == Request::Disabled &&
          controller.status().fault == Fault::NotArmed,
        "startup automatic selection cannot bypass DOWN arming");

  arm(controller, input);
  input.fold_ready = false;
  check(controller.update(input) == Request::Disabled &&
          controller.status().state == State::WaitReady, "missing calibration waits at zero output");
  input.now_ms = 50000U;
  input.fold_ready = true;
  input.motors_ready = false;
  check(controller.update(input) == Request::Disabled && controller.status().started_ms == 0U,
        "missing feedback also waits without consuming the timer");
  input.motors_ready = true;
  check(controller.update(input) == Request::Fold && controller.status().started_ms == 50000U &&
          controller.status().fold_elapsed_ms == 0U, "ready feedback starts a fresh fold interval");
  input.now_ms = 50000U + mission_config::kFoldDurationMs - 1U;
  check(controller.update(input) == Request::Fold &&
          controller.status().fold_elapsed_ms == mission_config::kFoldDurationMs - 1U,
        "one millisecond before the deadline still requests folding");
  ++input.now_ms;
  check(controller.update(input) == Request::Tracking &&
          controller.status().fold_elapsed_ms == mission_config::kFoldDurationMs,
        "the exact 7 second deadline requests tracking without adding the fold hold");
  input.now_ms += 50000U;
  check(controller.update(input) == Request::Tracking,
        "tracking remains selected instead of restarting the fold timer");

  arm(controller, input);
  check(controller.status().state == State::Disabled && controller.status().started_ms == 0U &&
          controller.status().fold_elapsed_ms == 0U, "DOWN clears all timer history");
  check(controller.update(input) == Request::Fold && controller.status().fold_elapsed_ms == 0U,
        "raising the right switch restarts with folding, not tracking");
  input.selected = false;
  check(controller.update(input) == Request::Disabled &&
          controller.status().state == State::Disabled, "leaving auto releases control to manual");
  input.selected = true;
  check(controller.update(input) == Request::Fold && controller.status().fold_elapsed_ms == 0U,
        "normal automatic reentry starts a fresh interval instead of resuming tracking");

  arm(controller, input);
  check(controller.update(input) == Request::Fold, "restart after acknowledgment");
  input.remote_online = false;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::RemoteLost,
        "remote loss aborts automatic operation");
  input.remote_online = true;
  input.selected = false;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::RemoteLost,
        "manual selection cannot clear an automatic fault");
  input.selected = true;
  arm(controller, input);
  controller.update(input);
  input.motors_ready = false;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::FeedbackLost,
        "feedback lost after start aborts rather than returning to readiness wait");
  input.motors_ready = true;
  arm(controller, input);
  controller.update(input);
  input.control_fault = true;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::ControlFault,
        "a controller fault blocks the timed handover");

  // Emergency DOWN wins even if every other supplied condition is unfavorable.
  input.stop_requested = true;
  input.gap_ms = 100U;
  input.motors_ready = input.fold_ready = false;
  arm(controller, input);
  input.control_fault = input.stop_requested = false;
  input.motors_ready = input.fold_ready = true;
  input.gap_ms = 1U;
  controller.update(input);
  input.stop_requested = true;
  input.now_ms += mission_config::kFoldDurationMs;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::ProgramStop,
        "program stop takes priority over the 7 second deadline");
  input.stop_requested = false;
  arm(controller, input);
  controller.update(input);
  input.gap_ms = mission_config::kMaxControlGapMs + 1U;
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::Timing,
        "a scheduling stall cannot silently advance into tracking");
  controller.abort(Fault::ControlFault);
  check(controller.status().fault == Fault::Timing, "the first fault remains latched");
  input.gap_ms = 0U;
  arm(controller, input);
  check(controller.update(input) == Request::Disabled && controller.status().fault == Fault::Timing,
        "zero control interval is also rejected");

  input.gap_ms = 1U;
  input.now_ms = UINT32_MAX - 999U;
  arm(controller, input);
  check(controller.update(input) == Request::Fold, "fold starts before the millisecond counter wraps");
  input.now_ms = mission_config::kFoldDurationMs - 1001U;
  check(controller.update(input) == Request::Fold, "wraparound retains the pre-deadline interval");
  ++input.now_ms;
  check(controller.update(input) == Request::Tracking, "wraparound reaches the exact deadline");
  std::cout << "PASS: mission arming, readiness, 7s deadline, manual exit, disable priority, "
               "fault latching and timer wraparound\n";
}
