#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

sp::DBus remote(&huart3, false);
uint8_t dbus_connected = 0;

extern "C" void dbus_task()
{
  while (1) {
    remote.request();
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    dbus_connected = 1;
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    dbus_connected = 0;
    remote.request();
  }
}