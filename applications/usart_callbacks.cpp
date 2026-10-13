// UART回调统一分发: USART3用于遥控器, USART6用于激光测距。
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/laser/laser.hpp"

extern sp::DBus remote;
extern sp::LaserGY56 laser;

// 接收事件只交给对应设备处理; 遥控器续收, 激光模块解析后续收。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote.update(Size, osKernelSysTick());
    remote.request();
  }
  else if (huart == &huart6) {
    laser.receive_complete(Size);
  }
}

// UART出错后分别重启对应设备的接收, 不影响另一条串口。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == &huart6) {
    laser.start_receive();
  }
}
