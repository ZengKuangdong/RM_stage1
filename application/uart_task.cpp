#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::RM_Motor rm_motor1;
extern sp::PID rm_motor_pid_angle1;
extern sp::PID rm_motor_pid_speed1;
sp::Plotter plotter(&huart1, false);

extern "C" void uart_task(void * argument)
{
  while (1) {
    // plotter.plot(
    //   bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    // plotter.plot(
    //   bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    plotter.plot(
      rm_motor1.angle, rm_motor1.speed, rm_motor_pid_angle1.out, rm_motor_pid_speed1.out, 0.0f,
      0.0f);
    // char buffer[5] = {'H', 'e', 'l', 'l', 'o'};
    // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, sizeof(buffer), 10);

    // HAL_UART_Transmit(&huart1, (uint8_t *)bmi088.acc, sizeof(bmi088.acc), 10);
    // HAL_UART_Transmit(&huart1, (uint8_t *)bmi088.gyro, sizeof(bmi088.gyro), 10);
    osDelay(10);
  }
}
