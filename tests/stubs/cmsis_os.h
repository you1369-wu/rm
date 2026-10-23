#pragma once

#include <cstdint>

#ifdef TEST_OS_DELAY_EXTERNAL
int osDelay(uint32_t millisec);
#else
inline int osDelay(uint32_t) { return 0; }
#endif
