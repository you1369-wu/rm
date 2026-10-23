#include "imu_support.hpp"

#include <cmath>
#include <cstring>

#include "io/bmi088/bmi088_defs.h"

namespace
{
// Match the unchanged SP initialization: +/-6 g and +/-2000 degrees/s.
constexpr float kAccScale = 6.0f * 9.8f / 32768.0f;
constexpr float kGyroScale = 2000.0f / 32767.0f * 3.14159265358979323846f / 180.0f;
constexpr uint32_t kSpiTimeoutMs = 2;

int16_t little_endian_i16(const uint8_t * data)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}
}  // namespace

namespace app
{
Bmi088WithStatus::Bmi088WithStatus(
  SPI_HandleTypeDef * spi, GPIO_TypeDef * acc_port, uint16_t acc_pin,
  GPIO_TypeDef * gyro_port, uint16_t gyro_pin, const float rotation[3][3])
: sp::BMI088(spi, acc_port, acc_pin, gyro_port, gyro_pin, rotation),
  spi_(spi), acc_port_(acc_port), gyro_port_(gyro_port), acc_pin_(acc_pin), gyro_pin_(gyro_pin)
{
  std::memcpy(rotation_, rotation, sizeof(rotation_));
  for (unsigned i = 0; i < 3; ++i) acc[i] = gyro[i] = 0.0f;
  temp = 0.0f;
}

bool Bmi088WithStatus::read(
  GPIO_TypeDef * port, uint16_t pin, uint8_t reg, uint8_t * data, uint16_t size,
  bool accelerometer)
{
  uint8_t tx[8] = {}, rx[8] = {};
  const uint16_t prefix = accelerometer ? 2U : 1U;
  if (!spi_ || !port || !data || size + prefix > sizeof(tx)) return false;
  tx[0] = reg | 0x80U;
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  const auto status = HAL_SPI_TransmitReceive(spi_, tx, rx, size + prefix, kSpiTimeoutMs);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  if (status != HAL_OK) return false;
  std::memcpy(data, rx + prefix, size);
  return true;
}

void Bmi088WithStatus::update()
{
  update_ok_ = false;
  uint8_t acc_id = 0, gyro_id = 0, acc_raw[6] = {}, gyro_raw[6] = {}, temp_raw[2] = {};
  // A disconnected SPI slave can still return HAL_OK. Verify both chip IDs as well.
  if (!read(acc_port_, acc_pin_, BMI088_ACC_CHIP_ID, &acc_id, 1, true) ||
      acc_id != BMI088_ACC_CHIP_ID_VALUE ||
      !read(gyro_port_, gyro_pin_, BMI088_GYRO_CHIP_ID, &gyro_id, 1, false) ||
      gyro_id != BMI088_GYRO_CHIP_ID_VALUE ||
      !read(acc_port_, acc_pin_, BMI088_ACC_DATA, acc_raw, 6, true) ||
      !read(gyro_port_, gyro_pin_, BMI088_GYRO_DATA, gyro_raw, 6, false) ||
      !read(acc_port_, acc_pin_, BMI088_TEMP_DATA, temp_raw, 2, true)) return;

  float sensor_acc[3], sensor_gyro[3], next_acc[3] = {}, next_gyro[3] = {};
  for (unsigned i = 0; i < 3; ++i) {
    sensor_acc[i] = little_endian_i16(acc_raw + 2 * i) * kAccScale;
    sensor_gyro[i] = little_endian_i16(gyro_raw + 2 * i) * kGyroScale;
  }
  int16_t raw_temp = (static_cast<uint16_t>(temp_raw[0]) << 3) | (temp_raw[1] >> 5);
  if (raw_temp > 1023) raw_temp -= 2048;
  const float next_temp = raw_temp * 0.125f + 23.0f;
  for (unsigned row = 0; row < 3; ++row) {
    for (unsigned col = 0; col < 3; ++col) {
      next_acc[row] += rotation_[row][col] * sensor_acc[col];
      next_gyro[row] += rotation_[row][col] * sensor_gyro[col];
    }
    if (!std::isfinite(next_acc[row]) || !std::isfinite(next_gyro[row])) return;
  }
  if (!std::isfinite(next_temp)) return;
  std::memcpy(acc, next_acc, sizeof(next_acc));
  std::memcpy(gyro, next_gyro, sizeof(next_gyro));
  temp = next_temp;
  update_ok_ = true;
}

TimedMahony::TimedMahony(float kp, float ki) : sp::Mahony(kStepS, kp, ki)
{
  yaw = pitch = roll = vyaw = vpitch = vroll = 0.0f;
  pitch_geom = pitch_geom_last = vpitch_geom = 0.0f;
  for (unsigned i = 0; i < 3; ++i) w[i] = acc_euler[i] = 0.0f;
  for (unsigned i = 0; i < 4; ++i) q[i] = q_last[i] = dq[i] = i == 0 ? 1.0f : 0.0f;
}

bool TimedMahony::set_dt(float dt)
{
  period_ok_ = std::isfinite(dt) && dt > 0.0f && dt <= 5.0f * kStepS;
  if (period_ok_)
    elapsed_s_ = dt;
  else
    remainder_s_ = 0.0f;
  return period_ok_;
}

bool TimedMahony::update(const float acc[3], const float gyro[3])
{
  if (!period_ok_ || !acc || !gyro) return false;
  float norm2 = 0.0f;
  for (unsigned i = 0; i < 3; ++i) {
    if (!std::isfinite(acc[i]) || !std::isfinite(gyro[i])) return false;
    norm2 += acc[i] * acc[i];
  }
  if (!std::isfinite(norm2) || norm2 <= 0.0f) return false;
  remainder_s_ += elapsed_s_;
  const unsigned steps = static_cast<unsigned>((remainder_s_ + 1.0e-8f) / kStepS);
  remainder_s_ -= steps * kStepS;
  if (remainder_s_ < 0.0f) remainder_s_ = 0.0f;
  for (unsigned i = 0; i < steps; ++i) sp::Mahony::update(acc, gyro);
  return steps != 0 && std::isfinite(yaw) && std::isfinite(pitch) && std::isfinite(roll);
}
}  // namespace app
