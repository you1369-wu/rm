// 折叠专用重力补偿计算
#ifndef APPLICATIONS_FOLD_MODE_GRAVITY_COMPENSATION_HPP
#define APPLICATIONS_FOLD_MODE_GRAVITY_COMPENSATION_HPP

#include <cmath>

namespace yuntai
{
// Same model as feature/motor_b_pid. Positive delta lifts +X toward +Z.
inline float calculate_gravity_compensation(float delta_rad)
{
  constexpr float mass_kg = 0.229f;  // Tuned equivalent mass: 179 g + 50 g.
  constexpr float gravity_m_s2 = 9.81f;
  // CAD center of mass minus pivot; mm to m. Rotation axis is Y.
  constexpr float com_x_m = (-161.414f - (-173.07f)) * 0.001f;
  constexpr float com_z_m = (954.657f - 1052.06f) * 0.001f;

  const float lever_arm_m = com_x_m * std::cos(delta_rad) - com_z_m * std::sin(delta_rad);
  return mass_kg * gravity_m_s2 * lever_arm_m;
}
}  // namespace yuntai

#endif  // APPLICATIONS_FOLD_MODE_GRAVITY_COMPENSATION_HPP
