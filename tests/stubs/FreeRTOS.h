#pragma once
#include <cstdint>
using TickType_t = uint32_t;
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
// Host tests are single-threaded. These stubs do not test IRQ exclusion.
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
