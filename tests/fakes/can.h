#ifndef TESTS_FAKES_CAN_H
#define TESTS_FAKES_CAN_H

#include <cstddef>
#include <cstdint>

struct CAN_HandleTypeDef {};
struct CAN_FilterTypeDef {};
constexpr uint32_t CAN_RX_FIFO0 = 0U;

extern "C" {
extern CAN_HandleTypeDef hcan1;
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef * hcan, uint32_t fifo);
}

#endif
