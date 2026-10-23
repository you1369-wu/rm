#pragma once
#include "tracking_config.hpp"
namespace tracking {
struct Reference { float position, velocity, acceleration; };
struct AxisOutput {
  float error=0, speed_command=0, acceleration_command=0;
  float pid_torque=0, feedforward_torque=0, torque=0;
  bool saturated=false;
};
class AxisController {
public:
  explicit AxisController(const tracking_config::AxisGains &gains): gains_(gains) {}
  AxisOutput step(Reference r, float position, float velocity, float dt, float speed_limit=0);
  void reset();
  float integral() const { return integral_; }
private:
  tracking_config::AxisGains gains_;
  float integral_=0, previous_speed_=0, filtered_acceleration_=0;
  bool derivative_ready_=false;
};
Reference approach(float start,float end,float elapsed_s,float duration_s);
bool finite(Reference r);
}
