// 在失能下档用 USART1 打印两台电机的角度，供机械位置标定。
#include "usart_task.h"

#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "remote_task.h"
#include "usart.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

namespace
{
constexpr uint32_t kPrintPeriodMs = 100U;
constexpr uint32_t kUartTimeoutMs = 20U;
constexpr long kFixedPointScale = 1000L;

char tx_buffer[64];

long float_to_milli(float value)
{
  const float scaled = value * static_cast<float>(kFixedPointScale);
  return static_cast<long>((scaled >= 0.0F) ? (scaled + 0.5F) : (scaled - 0.5F));
}

std::size_t append_text(char * buffer, std::size_t size, std::size_t offset, const char * text)
{
  const int written = std::snprintf(buffer + offset, size - offset, "%s", text);
  if (written <= 0) {
    return offset;
  }

  const std::size_t remaining = size - offset;
  if (static_cast<std::size_t>(written) >= remaining) {
    return size - 1U;
  }

  return offset + static_cast<std::size_t>(written);
}

std::size_t append_fixed(char * buffer, std::size_t size, std::size_t offset, float value)
{
  long scaled = float_to_milli(value);
  const char * sign = "";
  if (scaled < 0L) {
    sign = "-";
    scaled = -scaled;
  }

  const int written = std::snprintf(
    buffer + offset, size - offset, "%s%ld.%03ld", sign, scaled / kFixedPointScale,
    scaled % kFixedPointScale);
  if (written <= 0) {
    return offset;
  }

  const std::size_t remaining = size - offset;
  if (static_cast<std::size_t>(written) >= remaining) {
    return size - 1U;
  }

  return offset + static_cast<std::size_t>(written);
}

std::size_t build_angle_line(char * buffer, std::size_t size, float angle_a, float angle_b)
{
  std::size_t offset = 0U;

  offset = append_text(buffer, size, offset, "motor_a_rad:");
  offset = append_fixed(buffer, size, offset, angle_a);
  offset = append_text(buffer, size, offset, " motor_b_rad:");
  offset = append_fixed(buffer, size, offset, angle_b);
  offset = append_text(buffer, size, offset, "\r\n");

  return offset;
}
}  // namespace

extern "C" void usart_task(void const * argument)
{
  (void)argument;

  while (true) {
    RemoteTaskData remote_data = {};
    const uint32_t now_ms = osKernelSysTick();

    // 仅在遥控器下档且反馈未超时时输出，避免把旧角度误当成标定数据。
    if (remote_task_get_data(&remote_data) && remote_data.is_alive &&
        remote_data.sw_r == REMOTE_SWITCH_DOWN && motor_a.is_alive(now_ms) &&
        motor_b.is_alive(now_ms)) {
      const float angle_a = motor_a.angle;
      const float angle_b = motor_b.angle;
      if (std::isfinite(angle_a) && std::isfinite(angle_b)) {
        const std::size_t length = build_angle_line(tx_buffer, sizeof(tx_buffer), angle_a, angle_b);
        HAL_UART_Transmit(
          &huart1, reinterpret_cast<uint8_t *>(tx_buffer), static_cast<uint16_t>(length),
          kUartTimeoutMs);
      }
    }

    osDelay(kPrintPeriodMs);
  }
}
