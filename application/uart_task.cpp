#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

extern sp::BMI088 bmi088;
//sp::Plotter plotter(&huart1, false);

extern "C" void uart_task()
{
  while (1) {
    // plotter.plot(
    //   bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    HAL_UART_Transmit(&huart1, (uint8_t *)bmi088.acc, sizeof(bmi088.acc), 10);
    HAL_UART_Transmit(&huart1, (uint8_t *)bmi088.gyro, sizeof(bmi088.gyro), 10);
    osDelay(10);
  }
}
