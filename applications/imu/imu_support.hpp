#pragma once

#include <cstdint>

#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

namespace app
{
// Reuse SP initialization, but check every runtime SPI read before publishing a sample.
class Bmi088WithStatus : public sp::BMI088
{
public:
  Bmi088WithStatus(
    SPI_HandleTypeDef * spi, GPIO_TypeDef * acc_port, uint16_t acc_pin,
    GPIO_TypeDef * gyro_port, uint16_t gyro_pin, const float rotation[3][3]);

  // Input: none. Output: acc/gyro/temp change only after all reads succeed.
  void update();
  // True means the last complete sample passed SPI, chip-ID and finite-value checks.
  bool last_update_ok() const { return update_ok_; }

private:
  bool read(
    GPIO_TypeDef * port, uint16_t pin, uint8_t reg, uint8_t * data, uint16_t size,
    bool accelerometer);

  SPI_HandleTypeDef * spi_;
  GPIO_TypeDef * acc_port_;
  GPIO_TypeDef * gyro_port_;
  uint16_t acc_pin_, gyro_pin_;
  float rotation_[3][3];
  bool update_ok_ = false;
};

// SP keeps dt private and const. Retain its filter state and integrate elapsed time
// in 1 ms substeps instead of reconstructing it or modifying its private storage.
class TimedMahony : public sp::Mahony
{
public:
  explicit TimedMahony(float kp = 0.5f, float ki = 0.0f);
  // Input: elapsed seconds in (0, 0.005]. False rejects an invalid control interval.
  bool set_dt(float dt);
  // Input: m/s^2 and rad/s. True means at least one integration step completed.
  bool update(const float acc[3], const float gyro[3]);

private:
  static constexpr float kStepS = 0.001f;
  float elapsed_s_ = kStepS;
  float remainder_s_ = 0.0f;
  bool period_ok_ = true;
};
}  // namespace app
