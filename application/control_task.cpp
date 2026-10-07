#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

constexpr uint8_t HIST_LEN = 15;        // 与 15ms 前比较
constexpr float IMU_MOVE_TH = 0.02f;    // C板 15ms 净位移，约 0.11°（约 7.6°/s）
constexpr float MOTOR_MOVE_TH = 0.02f;  // 电机 15ms 净位移，约 0.11°
constexpr float MANUAL_TH = 0.05f;      // 电机离开目标，约 3°
constexpr float SETTLE_TH = 0.03f;      // 跟随电机到达目标的判定门槛
constexpr uint8_t SOURCE_RELEASE_MS = 30;
constexpr float RESET_A_OFFSET = 0.0f;  // A箭头相对编码器零点的标定偏移
constexpr float RESET_B_OFFSET = 0.0f;  // B箭头相对编码器零点的标定偏移

enum class InputSource
{
  NONE,
  BOARD,
  HAND_A,
  HAND_B
};

extern sp::DBus remote;
extern sp::Mahony imu;
sp::CAN can(&hcan1);
sp::RM_Motor rm_motor1(1, sp::RM_Motors::GM6020);
sp::RM_Motor rm_motor2(2, sp::RM_Motors::GM6020);
sp::AngleUnwrapper yaw_unwapper;

// sp::PID rm_motor_pid_angle1(0.001f, 0.5f, 0.0f, 0.0f, 1, 0.0f, 1.0f, false, true);
// sp::PID rm_motor_pid_speed1(0.001f, 0.5f, 0.0f, 0.0f, 1, 0.0f, 1.0f, false, true);

// sp::PID rm_motor_pid_angle2(0.001f, 0.5f, 0.0f, 0.0f, 1, 0.0f, 1.0f, false, true);
// sp::PID rm_motor_pid_speed2(0.001f, 0.5f, 0.0f, 0.0f, 1, 0.0f, 1.0f, false, true);

// // 角度环: rad → rad/s    调试的参数还算能看
// sp::PID rm_motor_pid_angle1(0.001f, 2.0f, 2.0f, 0.0f, 1.5f, 1.5f, 1.0f, false, false);
// sp::PID rm_motor_pid_angle2(0.001f, 2.0f, 2.0f, 0.0f, 1.5f, 1.5f, 1.0f, false, false);
// // 速度环: rad/s → N·m
// sp::PID rm_motor_pid_speed1(0.001f, 0.02f, 2.0f, 0.0f, 0.15f, 0.115f, 1.0f, false, false);
// sp::PID rm_motor_pid_speed2(0.001f, 0.02f, 2.0f, 0.0f, 0.15f, 0.135f, 1.0f, false, false);

// 角度环: rad → rad/s
sp::PID rm_motor_pid_angle1(0.001f, 3.0f, 2.0f, 0.0f, 1.5f, 1.5f, 1.0f, false, false);
sp::PID rm_motor_pid_angle2(0.001f, 3.0f, 2.0f, 0.0f, 1.5f, 1.5f, 1.0f, false, false);
// 速度环: rad/s → N·m
sp::PID rm_motor_pid_speed1(0.001f, 0.04f, 2.0f, 0.0f, 0.15f, 0.115f, 1.0f, false, false);
sp::PID rm_motor_pid_speed2(0.001f, 0.04f, 2.0f, 0.0f, 0.15f, 0.135f, 1.0f, false, false);

extern "C" void pid_control_clac(float set, uint8_t motor_id);
extern "C" void pid_control_send();
extern "C" void hist_push(float a[], float b[], float c[], uint8_t len, float yaw);

static float get_b_ratio(sp::DBusSwitchMode mode)
{
  if (mode == sp::DBusSwitchMode::DOWN) return 0.5f;  //0.5
  if (mode == sp::DBusSwitchMode::MID) return -1.0f;  //-1
  return 3.0f;                                        //3
}

static void disable_motors()
{
  rm_motor1.cmd(0.0f);
  rm_motor2.cmd(0.0f);
  rm_motor1.write(can.tx_data);
  rm_motor2.write(can.tx_data);
  rm_motor_pid_angle1.clear();
  rm_motor_pid_speed1.clear();
  rm_motor_pid_angle2.clear();
  rm_motor_pid_speed2.clear();
}

static void release_motor(uint8_t motor_id)
{
  if (motor_id == 1) {
    rm_motor1.cmd(0.0f);
    rm_motor1.write(can.tx_data);
    rm_motor_pid_angle1.clear();
    rm_motor_pid_speed1.clear();
  }
  else if (motor_id == 2) {
    rm_motor2.cmd(0.0f);
    rm_motor2.write(can.tx_data);
    rm_motor_pid_angle2.clear();
    rm_motor_pid_speed2.clear();
  }
}

static float nearest_equivalent_angle(float desired, float current)
{
  return current + sp::limit_angle(desired - current);
}

extern "C" void control_task(void * argument)
{
  can.config();
  can.start();

  float yaw = yaw_unwapper.update(imu.yaw);
  float imu_ref = yaw;
  float rm_motor1_ref = rm_motor1.angle;
  float rm_motor2_ref = rm_motor2.angle;

  float imu_hist[HIST_LEN] = {0};
  float rm_motor1_hist[HIST_LEN] = {0};
  float rm_motor2_hist[HIST_LEN] = {0};
  uint8_t hist_fill = 0;

  sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::DOWN;
  uint8_t last_left_flag = 0;
  InputSource input_source = InputSource::NONE;
  uint8_t source_release_ms = 0;

  while (1) {
    uint32_t now = osKernelSysTick();
    yaw = yaw_unwapper.update(imu.yaw);

    const bool remote_ok = remote.is_alive(now);
    const bool motors_ok = rm_motor1.is_alive(now) && rm_motor2.is_alive(now);
    const bool disable = !remote_ok || !motors_ok || remote.sw_r == sp::DBusSwitchMode::DOWN;

    if (disable) {
      disable_motors();
      last_sw_r = sp::DBusSwitchMode::DOWN;
      last_left_flag = 0;
      hist_fill = 0;
      input_source = InputSource::NONE;
      source_release_ms = 0;
    }
    else if (remote.sw_r == sp::DBusSwitchMode::MID) {
      const float k_b = get_b_ratio(remote.sw_l);
      uint8_t left_flag = 3;
      if (remote.sw_l == sp::DBusSwitchMode::UP)
        left_flag = 1;
      else if (remote.sw_l == sp::DBusSwitchMode::DOWN)
        left_flag = 2;

      // 刚进入中档，或左拨杆换挡：把当前位置当作新零点，避免回拉/跳变
      if (last_sw_r != sp::DBusSwitchMode::MID || left_flag != last_left_flag) {
        imu_ref = yaw;
        rm_motor1_ref = rm_motor1.angle;
        rm_motor2_ref = rm_motor2.angle;
        last_left_flag = left_flag;
        hist_fill = 0;
        input_source = InputSource::NONE;
        source_release_ms = 0;
      }

      const float q = yaw - imu_ref;
      float target_a = rm_motor1_ref + q;
      float target_b = rm_motor2_ref + k_b * q;

      const float yaw_old = (hist_fill >= HIST_LEN) ? imu_hist[0] : yaw;
      const bool imu_moved = (hist_fill >= HIST_LEN) && (std::fabs(yaw - yaw_old) > IMU_MOVE_TH);
      const bool motor1_moved =
        (hist_fill >= HIST_LEN) && (std::fabs(rm_motor1.angle - rm_motor1_hist[0]) > MOTOR_MOVE_TH);
      const bool motor2_moved =
        (hist_fill >= HIST_LEN) && (std::fabs(rm_motor2.angle - rm_motor2_hist[0]) > MOTOR_MOVE_TH);

      const float err_a = std::fabs(rm_motor1.angle - target_a);
      const float err_b = std::fabs(rm_motor2.angle - target_b);
      const bool hand_a = err_a > MANUAL_TH;
      const bool hand_b = err_b > MANUAL_TH;
      const float k_b_abs = std::fabs(k_b);

      // C板转动优先级最高；手拧状态锁定后，另一台跟随电机不能抢占输入源。
      if (imu_moved) {
        input_source = InputSource::BOARD;
        source_release_ms = 0;
      }
      else if (input_source == InputSource::NONE) {
        if (hand_a && (k_b_abs < 1e-3f || err_a >= err_b / k_b_abs))
          input_source = InputSource::HAND_A;
        else if (hand_b && k_b_abs > 1e-3f)
          input_source = InputSource::HAND_B;
      }

      if (input_source == InputSource::HAND_A) {
        const float q_hand = rm_motor1.angle - rm_motor1_ref;
        imu_ref = yaw - q_hand;
        target_a = rm_motor1.angle;
        target_b = rm_motor2_ref + k_b * q_hand;
        release_motor(1);

        if (!motor1_moved && std::fabs(rm_motor2.angle - target_b) < SETTLE_TH) {
          if (source_release_ms < SOURCE_RELEASE_MS) source_release_ms++;
          if (source_release_ms >= SOURCE_RELEASE_MS) input_source = InputSource::NONE;
        }
        else {
          source_release_ms = 0;
        }
      }
      else if (input_source == InputSource::HAND_B) {
        const float q_hand = (rm_motor2.angle - rm_motor2_ref) / k_b;
        imu_ref = yaw - q_hand;
        target_a = rm_motor1_ref + q_hand;
        target_b = rm_motor2.angle;
        release_motor(2);

        if (!motor2_moved && std::fabs(rm_motor1.angle - target_a) < SETTLE_TH) {
          if (source_release_ms < SOURCE_RELEASE_MS) source_release_ms++;
          if (source_release_ms >= SOURCE_RELEASE_MS) input_source = InputSource::NONE;
        }
        else {
          source_release_ms = 0;
        }
      }
      else if (input_source == InputSource::BOARD) {
        if (!imu_moved && err_a < SETTLE_TH && err_b < SETTLE_TH) {
          if (source_release_ms < SOURCE_RELEASE_MS) source_release_ms++;
          if (source_release_ms >= SOURCE_RELEASE_MS) input_source = InputSource::NONE;
        }
        else {
          source_release_ms = 0;
        }
      }

      if (input_source != InputSource::HAND_A) pid_control_clac(target_a, 1);
      if (input_source != InputSource::HAND_B) pid_control_clac(target_b, 2);

      last_left_flag = left_flag;
      last_sw_r = sp::DBusSwitchMode::MID;
    }
    else if (remote.sw_r == sp::DBusSwitchMode::UP) {
      const float target_a = nearest_equivalent_angle(yaw + RESET_A_OFFSET, rm_motor1.angle);
      const float target_b = nearest_equivalent_angle(yaw + RESET_B_OFFSET, rm_motor2.angle);
      pid_control_clac(target_a, 1);
      pid_control_clac(target_b, 2);
      last_left_flag = 0;
      last_sw_r = sp::DBusSwitchMode::UP;
      input_source = InputSource::NONE;
      source_release_ms = 0;
      hist_fill = 0;
    }

    hist_push(imu_hist, rm_motor1_hist, rm_motor2_hist, HIST_LEN, yaw);
    if (hist_fill < HIST_LEN) hist_fill++;

    pid_control_send();
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
// 单速度环pid
//			rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, rm_motor_x.speed);
//			rm_motor_data.given_torque = rm_motor_pid_speed.out;
extern "C" void pid_control_clac(float set, uint8_t motor_id)
{
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

extern "C" void hist_push(float a[], float b[], float c[], uint8_t len, float yaw)
{
  for (uint8_t i = 0; i < len - 1; i++) {
    a[i] = a[i + 1];
    b[i] = b[i + 1];
    c[i] = c[i + 1];
  }
  a[len - 1] = yaw;
  b[len - 1] = rm_motor1.angle;
  c[len - 1] = rm_motor2.angle;
}
