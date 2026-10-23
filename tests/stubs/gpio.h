#pragma once

#include <cstdint>

struct GPIO_TypeDef { unsigned id; };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };

extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef * port, uint16_t pin, GPIO_PinState state);
