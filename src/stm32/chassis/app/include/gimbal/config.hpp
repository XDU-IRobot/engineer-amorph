#pragma once

#include <librm.hpp>

#include "m_pi_f.hpp"

struct GimbalConfig {
  struct PDParams {
    float kp;
    float kd;
  };

  float control_loop_dt;  ///< 控制循环周期，单位秒
  uint8_t imu_node_id;
  uint16_t yaw_motor_id;
  rm::modules::MotorCalibrator::MotorConfig yaw_calib;  ///< 前舵电机校准配置
  int servo_mid_angle;
  int servo_move_time_ms;
  PDParams lift_pd_param;

  static GimbalConfig Make() {
    return {.control_loop_dt = 1.f / 250.f,
            .imu_node_id = 8,
            .yaw_motor_id = 7,
            .yaw_calib = {.reverse = true, .zero_offset = 4936.f / 8191.f * M_PIf * 2.f},
            .servo_mid_angle = 600,
            .servo_move_time_ms = 350,
            .lift_pd_param = {300.f, 15.f}};
  }
};