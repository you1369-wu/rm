#pragma once
#include "tracking_config.hpp"

namespace tracking
{
struct Reference { float position, velocity, acceleration; };
struct AxisOutput
{
  float error = 0.0f;
  float speed_command = 0.0f;
  float torque = 0.0f;
  bool saturated = false;
};
// 独立于HAL的控制核，便于主机测试。限幅作用于含前馈的最终输出。
class AxisController
{
public:
  explicit AxisController(const tracking_config::AxisGains & gains) : gains_(gains) {}
  AxisOutput step(Reference reference, float position, float velocity, float dt);
  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }
private:
  tracking_config::AxisGains gains_;
  float integral_ = 0.0f;
};
Reference approach(float start, float end, float elapsed_s, float duration_s);
bool finite(Reference reference);
} // namespace tracking
