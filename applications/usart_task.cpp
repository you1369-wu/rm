// USART1 is owned by plot_task. Keep the CubeMX-generated task entry available.
#include "usart_task.h"
#include "cmsis_os.h"

extern "C" void usart_task(void const * argument)
{
  (void)argument;
  for (;;) osDelay(1000);
}
