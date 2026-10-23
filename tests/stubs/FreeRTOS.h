#pragma once
#include <cstdint>
using TickType_t = uint32_t;
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
// Tests alternate control and trajectory at a condition-variable barrier.
// These stubs do not model interrupt masking or preemption.
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
