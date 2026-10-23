#pragma once

// Organizer scoring API. Angles are radians; call once per 1 ms test cycle.
void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio);
