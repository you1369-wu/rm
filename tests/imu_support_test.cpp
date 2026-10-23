#include "imu/imu_support.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

#include "io/bmi088/bmi088_defs.h"

namespace
{
SPI_HandleTypeDef spi;
GPIO_TypeDef acc_port{1}, gyro_port{2};
GPIO_TypeDef * selected = nullptr;
uint8_t registers[2][128] = {};
unsigned transfers = 0, fail_at = 0;
HAL_StatusTypeDef failure = HAL_ERROR;

void set_i16(unsigned sensor, unsigned reg, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  registers[sensor][reg] = raw & 0xffU;
  registers[sensor][reg + 1] = raw >> 8;
}

void reset_bus()
{
  std::memset(registers, 0, sizeof(registers));
  selected = nullptr;
  transfers = fail_at = 0;
  registers[0][BMI088_ACC_CHIP_ID] = BMI088_ACC_CHIP_ID_VALUE;
  registers[1][BMI088_GYRO_CHIP_ID] = BMI088_GYRO_CHIP_ID_VALUE;
  const int16_t values[3] = {1000, -2000, 16384};
  for (unsigned i = 0; i < 3; ++i) {
    set_i16(0, BMI088_ACC_DATA + 2 * i, values[i]);
    set_i16(1, BMI088_GYRO_DATA + 2 * i, values[i]);
  }
  // Signed 11-bit temperature -8 gives 22 degrees C.
  const uint16_t temperature = static_cast<uint16_t>(-8) & 0x7ffU;
  registers[0][BMI088_TEMP_DATA] = temperature >> 3;
  registers[0][BMI088_TEMP_DATA + 1] = (temperature & 7U) << 5;
}

bool near(float a, float b) { return std::fabs(a - b) < 1.0e-5f; }

void test_sensor_status()
{
  reset_bus();
  const float rotation[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
  app::Bmi088WithStatus sensor(&spi, &acc_port, 1, &gyro_port, 2, rotation);
  assert(!sensor.last_update_ok());
  sensor.init();
  sensor.update();
  assert(sensor.last_update_ok());
  const float acc_scale = 6.0f * 9.8f / 32768.0f;
  const float gyro_scale = 2000.0f / 32767.0f * 3.14159265358979323846f / 180.0f;
  assert(near(sensor.acc[0], 2000 * acc_scale));
  assert(near(sensor.acc[1], 1000 * acc_scale));
  assert(near(sensor.acc[2], 16384 * acc_scale));
  assert(near(sensor.gyro[0], 2000 * gyro_scale));
  assert(near(sensor.gyro[1], 1000 * gyro_scale));
  assert(near(sensor.gyro[2], 16384 * gyro_scale));
  assert(near(sensor.temp, 22.0f));
  float old_acc[3], old_gyro[3];
  std::memcpy(old_acc, sensor.acc, sizeof(old_acc));
  std::memcpy(old_gyro, sensor.gyro, sizeof(old_gyro));

  const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
  for (unsigned transfer = 1; transfer <= 5; ++transfer) {
    for (auto status : failures) {
      reset_bus();
      set_i16(0, BMI088_ACC_DATA, 12345);
      fail_at = transfer;
      failure = status;
      sensor.update();
      assert(!sensor.last_update_ok());
      assert(selected == nullptr);
      assert(std::memcmp(sensor.acc, old_acc, sizeof(old_acc)) == 0);
      assert(std::memcmp(sensor.gyro, old_gyro, sizeof(old_gyro)) == 0);
      assert(near(sensor.temp, 22.0f));
    }
  }
  for (unsigned chip = 0; chip < 2; ++chip) {
    reset_bus();
    registers[chip][0] = 0xff;
    sensor.update();
    assert(!sensor.last_update_ok());
    assert(selected == nullptr);
  }
  reset_bus();
  sensor.update();
  assert(sensor.last_update_ok());
}

void test_filter_timing()
{
  app::TimedMahony filter(0.5f, 0.1f);
  sp::Mahony reference(0.001f, 0.5f, 0.1f);
  reference.pitch_geom = reference.pitch_geom_last = 0.0f;
  const float level[3] = {0, 0, 9.8f};
  const float acc[3] = {0.8f, 0.4f, 9.7f};
  const float gyro[3] = {0.1f, 0.2f, 1.0f};
  assert(filter.update(level, gyro));
  reference.update(level, gyro);
  for (unsigned n = 0; n < 30; ++n) {
    const unsigned steps = n % 5 + 1;
    assert(filter.set_dt(steps * 0.001f));
    assert(filter.update(acc, gyro));
    for (unsigned i = 0; i < steps; ++i) reference.update(acc, gyro);
    for (unsigned i = 0; i < 4; ++i) assert(near(filter.q[i], reference.q[i]));
  }
  const float old_yaw = filter.yaw;
  const float invalid[] = {
    0, -0.001f, 0.006f, std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN()};
  for (auto dt : invalid) {
    assert(!filter.set_dt(dt));
    assert(!filter.update(acc, gyro));
    assert(near(filter.yaw, old_yaw));
  }
  assert(filter.set_dt(0.0005f));
  assert(!filter.update(acc, gyro));
  assert(filter.update(acc, gyro));
  reference.update(acc, gyro);
  for (unsigned i = 0; i < 4; ++i) assert(near(filter.q[i], reference.q[i]));
  assert(filter.set_dt(0.001f));
  const float zero[3] = {};
  assert(!filter.update(zero, gyro));
  float bad_gyro[3] = {0, 0, std::numeric_limits<float>::quiet_NaN()};
  assert(!filter.update(acc, bad_gyro));
  assert(!filter.update(nullptr, gyro));
  assert(filter.update(acc, gyro));
  reference.update(acc, gyro);
  for (unsigned i = 0; i < 4; ++i) assert(near(filter.q[i], reference.q[i]));
}
}  // namespace

extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef * port, uint16_t pin, GPIO_PinState state)
{
  assert((port == &acc_port && pin == 1) || (port == &gyro_port && pin == 2));
  if (state == GPIO_PIN_RESET) {
    assert(selected == nullptr);
    selected = port;
  }
  else {
    assert(selected == port);
    selected = nullptr;
  }
}

extern "C" HAL_StatusTypeDef HAL_SPI_TransmitReceive(
  SPI_HandleTypeDef * bus, uint8_t * tx, uint8_t * rx, uint16_t size, uint32_t)
{
  assert(bus == &spi && selected != nullptr && (tx[0] & 0x80U));
  if (++transfers == fail_at) return failure;
  const unsigned sensor = selected == &acc_port ? 0 : 1;
  const unsigned prefix = sensor == 0 ? 2 : 1;
  assert(size >= prefix);
  std::memset(rx, 0xff, size);
  std::memcpy(rx + prefix, registers[sensor] + (tx[0] & 0x7fU), size - prefix);
  return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_SPI_Transmit(
  SPI_HandleTypeDef * bus, uint8_t * tx, uint16_t size, uint32_t)
{
  assert(bus == &spi && selected != nullptr && size == 2);
  const unsigned sensor = selected == &acc_port ? 0 : 1;
  registers[sensor][tx[0]] = tx[1];
  return HAL_OK;
}

int main()
{
  test_sensor_status();
  test_filter_timing();
  std::cout << "IMU SPI status and timed Mahony tests passed\n";
}
