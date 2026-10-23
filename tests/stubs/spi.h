#pragma once

#include <cstdint>

struct SPI_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };

extern "C" HAL_StatusTypeDef HAL_SPI_TransmitReceive(
  SPI_HandleTypeDef * spi, uint8_t * tx, uint8_t * rx, uint16_t size, uint32_t timeout);
extern "C" HAL_StatusTypeDef HAL_SPI_Transmit(
  SPI_HandleTypeDef * spi, uint8_t * tx, uint16_t size, uint32_t timeout);
