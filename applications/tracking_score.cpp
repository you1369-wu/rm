#include "tracking_score.hpp"

#include <cmath>
#include <cstdint>

void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) return;
  constexpr float alpha = 0.001f;
  constexpr float threshold = 0.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) return;
  constexpr float alpha = 0.001f;
  constexpr float threshold = 1.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}