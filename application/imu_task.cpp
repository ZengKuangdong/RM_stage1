#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/bmi088/bmi088_defs.h"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

extern "C" void imu_task()
{
  bmi088.init();
  while (true) {
    bmi088.update();
    osDelay(1);
  }
}