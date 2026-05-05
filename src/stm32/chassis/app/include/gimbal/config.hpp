#pragma once

#include <librm.hpp>

struct GimbalConfig {
  float control_loop_dt;  ///< 控制循环周期，单位秒
  uint8_t imu_node_id;
  uint16_t yaw_motor_id;
  rm::modules::MotorCalibrator::MotorConfig yaw_calib;  ///< 前舵电机校准配置
  int servo_mid_angle;
  int servo_move_time_ms;

  static GimbalConfig Make() {
    return {
        .control_loop_dt = 1.f / 250.f,
        .imu_node_id = 8,
        .yaw_motor_id = 7,
        .yaw_calib = {.reverse = true, .zero_offset = 5.9465191398212856179373381553497f},
        .servo_mid_angle = 600,
        .servo_move_time_ms = 350,
    };
  }
};