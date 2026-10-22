#ifndef TESTS_FAKES_CMSIS_OS_H
#define TESTS_FAKES_CMSIS_OS_H

#include <cstdint>

extern "C" {
uint32_t osKernelSysTick();
int osDelay(uint32_t milliseconds);
}

#endif
