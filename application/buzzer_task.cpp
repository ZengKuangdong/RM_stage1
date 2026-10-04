#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);
extern uint8_t dbus_connected;  // 遥控器连接状态 0:未连接, 1:已连接

extern "C" void buzzer_task(void * argument)
{
  buzzer.start();
  uint8_t nums = 0;
  while (1) {
    if (nums >= 3) {
      osDelay(1000);
    }
    else {
      uint16_t freq[7] = {262, 294, 330, 349, 392, 440, 494};  //中音区频率，高x2，低/2
      for (int i = 0; i < 7; i++) {
        buzzer.set(freq[i], 0.5);
        osDelay(200);
        nums++;
      }
      osDelay(1000);
    }
  }
  if (dbus_connected == 1) {
    buzzer.set(440, 0.5);
    dbus_connected = 0;
    osDelay(1000);
  }
}