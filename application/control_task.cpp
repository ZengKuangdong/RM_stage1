#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern sp::Mahony imu;  //imu_task中的bmi088实例
sp::CAN can(&hcan1);
sp::RM_Motor rm_motor1(1, sp::RM_Motors::GM6020);
sp::RM_Motor rm_motor2(2, sp::RM_Motors::GM6020);
sp::AngleUnwrapper yaw_unwapper;

sp::PID rm_motor_pid_angle1(0.001f, 1.0f, 1.0f, 1.0f, 1, 0.0f, 1.0f, false, true);
sp::PID rm_motor_pid_speed1(0.001f, 1.0f, 1.0f, 1.0f, 1, 0.0f, 1.0f, false, true);

sp::PID rm_motor_pid_angle2(0.001f, 1.0f, 1.0f, 1.0f, 1, 0.0f, 1.0f, false, true);
sp::PID rm_motor_pid_speed2(0.001f, 1.0f, 1.0f, 1.0f, 1, 0.0f, 1.0f, false, true);

extern "C" void pid_control_clac(float set, uint8_t motor_id)  //A为1，B为2
{
  // 双环pid
  if (motor_id == 1) {
    rm_motor_pid_angle1.calc(set, rm_motor1.angle);
    rm_motor_pid_speed1.calc(rm_motor_pid_angle1.out, rm_motor1.speed);

    rm_motor1.cmd(rm_motor_pid_speed1.out);
    rm_motor1.write(can.tx_data);
  }
  else if (motor_id == 2) {
    rm_motor_pid_angle2.calc(set, rm_motor2.angle);
    rm_motor_pid_speed2.calc(rm_motor_pid_angle2.out, rm_motor2.speed);

    rm_motor2.cmd(rm_motor_pid_speed2.out);
    rm_motor2.write(can.tx_data);
  }
}

extern "C" void pid_control_send() { can.send(rm_motor1.tx_id); }

extern "C" void control_task()
{
  can.config();
  can.start();
  can.recv();

  //   float init_yaw_angle = imu.yaw;

  //三输入零点
  float yaw = yaw_unwapper.update(imu.yaw);
  float imu_ref = yaw;
  float rm_motor1_ref = rm_motor1.angle;
  float rm_motor2_ref = rm_motor2.angle;

  uint8_t switch_flag = 0;  //上一次是什么挡
  uint8_t last_flag = 0;    //上次的开关状态，1为上档，2为下档，3为中档

  float imu_last = yaw;
  float rm_motor1_last = rm_motor1.angle;
  float rm_motor2_last = rm_motor2.angle;
  while (1) {
    if (
      remote.sw_r == sp::DBusSwitchMode::DOWN ||
      !remote.is_alive(osKernelSysTick())) {  //电机失能机制
      rm_motor1.cmd(0.0f);
      rm_motor1.write(can.tx_data);
      rm_motor2.cmd(0.0f);
      rm_motor2.write(can.tx_data);
      switch_flag = 0;
    }
    else if (remote.sw_r == sp::DBusSwitchMode::MID) {  //中挡姿态追踪

      yaw = yaw_unwapper.update(imu.yaw);
      float angle_to_turn = yaw - imu_ref;  //需要转过的角度
      float rm_motor1_angle = rm_motor1.angle - rm_motor1_ref;
      float rm_motor2_angle = rm_motor2.angle - rm_motor2_ref;
      float motor2_turn_param = 1.0f;

      if (remote.sw_l == sp::DBusSwitchMode::UP) {
        motor2_turn_param = 3.0f;
        if (last_flag != 1) {
          switch_flag = 1;
          last_flag = 1;
        }
      }
      else if (remote.sw_l == sp::DBusSwitchMode::DOWN) {
        motor2_turn_param = 0.5f;
        if (last_flag != 2) {
          switch_flag = 1;
          last_flag = 2;
        }
      }
      else if (remote.sw_l == sp::DBusSwitchMode::MID) {
        motor2_turn_param = -1.0f;
        if (last_flag != 3) {
          switch_flag = 1;
          last_flag = 3;
        }
      }

      if (switch_flag) {
        imu_ref = yaw;
        rm_motor1_ref = rm_motor1.angle;
        rm_motor2_ref = rm_motor2.angle;
        angle_to_turn = yaw - imu_ref;  //需要转过的角度
        rm_motor1_angle = rm_motor1.angle - rm_motor1_ref;
        rm_motor2_angle = rm_motor2.angle - rm_motor2_ref;
        switch_flag = 0;
      }

      if (yaw - imu_last > 0.0001f || yaw - imu_last < -0.0001f) {  //C板动
        pid_control_clac(rm_motor1_ref + angle_to_turn, 1);
        pid_control_clac(rm_motor2_ref + angle_to_turn * motor2_turn_param, 2);
      }
      else if (
        rm_motor1.angle - rm_motor1_last > 0.0001f ||
        rm_motor1.angle - rm_motor1_last < -0.0001f) {  //电机A动
        pid_control_clac(rm_motor2_ref + rm_motor1_angle * motor2_turn_param, 2);
        imu_ref -= rm_motor1.angle - rm_motor1_last;
      }
      else if (
        rm_motor2.angle - rm_motor2_last > 0.0001f ||
        rm_motor2.angle - rm_motor2_last < -0.0001f) {  //电机B动
        pid_control_clac(rm_motor1_ref + rm_motor2_angle / motor2_turn_param, 1);
        imu_ref -= (rm_motor2.angle - rm_motor2_last) / motor2_turn_param;
      }
    }
    else if (remote.sw_r == sp::DBusSwitchMode::UP) {  //上档复位
      pid_control_clac(yaw, 1);
      pid_control_clac(yaw, 2);
      switch_flag = 0;
    }

    pid_control_send();

    imu_last = yaw;
    rm_motor1_last = rm_motor1.angle;
    rm_motor2_last = rm_motor2.angle;
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can.recv();

      if (can.rx_id == rm_motor1.rx_id)
        rm_motor1.read(can.rx_data, stamp_ms);
      else if (can.rx_id == rm_motor2.rx_id)
        rm_motor2.read(can.rx_data, stamp_ms);
    }
  }
}