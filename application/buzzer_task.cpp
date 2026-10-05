#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "io/dbus/dbus.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);
extern sp::DBus remote;

extern "C" void buzzer_task()
{
  buzzer.start();
  uint8_t nums = 0;
  uint8_t dbus_connected_buzzer = 1;
  while (1) {
    if (nums >= 3) {
      osDelay(1000);
      buzzer.stop();
    }
    else {
      uint16_t freq[7] = {262, 294, 330, 349, 392, 440, 494};  //中音区频率，高x2，低/2
      for (int i = 0; i < 7; i++) {
        buzzer.set(freq[i], 0.5);
        osDelay(100);
      }
      nums++;
      osDelay(1000);
    }

    if (dbus_connected_buzzer && remote.is_open() && nums >= 3) {  //连接成功
      buzzer.set(440, 0.5);
      dbus_connected_buzzer = 0;
      osDelay(1000);
      buzzer.stop();
    }
    else if (!dbus_connected_buzzer && !remote.is_open() && nums >= 3) {  //断开连接
      buzzer.set(220, 0.5);
      osDelay(500);
      buzzer.set(110, 0.5);
      osDelay(500);
      dbus_connected_buzzer = 1;
      buzzer.stop();
    }
  }
}