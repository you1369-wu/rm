#include "tracking_controller.hpp"
#include <algorithm>
#include <cmath>
namespace tracking {
bool finite(Reference r) {
  return std::isfinite(r.position)&&std::isfinite(r.velocity)&&std::isfinite(r.acceleration);
}
void AxisController::reset() {
  integral_=previous_speed_=filtered_acceleration_=0; derivative_ready_=false;
}
AxisOutput AxisController::step(Reference r,float position,float velocity,float dt,float speed_limit) {
  if (!finite(r)||!std::isfinite(position)||!std::isfinite(velocity)||
      !std::isfinite(dt)||dt<=0||dt>.005f) { reset(); return {}; }
  AxisOutput out;
  out.error=r.position-position; // finite travel: do not wrap across mechanical stops
  const float limit=speed_limit>0?std::min(speed_limit,gains_.max_speed):gains_.max_speed;
  const float raw_speed=gains_.angle_kp*out.error+gains_.velocity_ff*r.velocity;
  out.speed_command=std::clamp(raw_speed,-limit,limit);
  // PDF: differentiate the limited speed-loop SETPOINT, not measured velocity.
  const float acceleration=derivative_ready_?(out.speed_command-previous_speed_)/dt:0;
  previous_speed_=out.speed_command; derivative_ready_=true;
  const float limited=std::clamp(acceleration,-gains_.max_acceleration,gains_.max_acceleration);
  filtered_acceleration_+=dt/(gains_.acceleration_filter_s+dt)*(limited-filtered_acceleration_);
  out.acceleration_command=filtered_acceleration_;
  out.feedforward_torque=gains_.inertia_ff*filtered_acceleration_+
    gains_.damping_ff*out.speed_command+gains_.gravity_ff*std::cos(position);
  const float error=out.speed_command-velocity, p=gains_.speed_kp*error;
  const float candidate=std::clamp(integral_+gains_.speed_ki*error*dt,
                                 -gains_.max_integral,gains_.max_integral);
  const float trial=p+candidate+out.feedforward_torque, increment=candidate-integral_;
  if (!((trial>gains_.max_torque&&increment>0)||(trial<-gains_.max_torque&&increment<0)))
    integral_=candidate;
  out.pid_torque=p+integral_;
  const float raw=out.pid_torque+out.feedforward_torque;
  out.torque=std::clamp(raw,-gains_.max_torque,gains_.max_torque);
  out.saturated=raw_speed!=out.speed_command||raw!=out.torque;
  return out;
}
Reference approach(float start,float end,float elapsed_s,float duration_s) {
  if (duration_s<=0) return {end,0,0};
  const float s=std::clamp(elapsed_s/duration_s,0.0f,1.0f), s2=s*s, s3=s2*s, d=end-start;
  return {start+d*(10*s3-15*s3*s+6*s3*s2),d*(30*s2-60*s3+30*s2*s2)/duration_s,
          d*(60*s-180*s2+120*s3)/(duration_s*duration_s)};
}
}
