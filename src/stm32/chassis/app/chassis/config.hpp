#pragma once

#include <librm.hpp>

struct ChassisConfig {
  float control_loop_dt;  ///< 控制循环周期，单位秒
  float kine_base_len;    ///< 运动学模型的底边长
  float kine_height;      ///< 运动学模型的等腰三角形的高

  // 电机id
  uint16_t f_steer_id;
  uint16_t l_steer_id;
  uint16_t r_steer_id;
  uint16_t f_wheel_id;
  uint16_t l_wheel_id;
  uint16_t r_wheel_id;
  rm::modules::MotorCalibrator::MotorConfig f_steer_calib;  ///< 前舵电机校准配置
  rm::modules::MotorCalibrator::MotorConfig l_steer_calib;  ///< 左舵电机校准配置
  rm::modules::MotorCalibrator::MotorConfig r_steer_calib;  ///< 右舵电机校准配置

  static ChassisConfig Make() {
    return {
        .control_loop_dt = 0.02f,
        .kine_base_len = 0.39f,
        .kine_height = 0.4f,
        .f_steer_id = 2,
        .l_steer_id = 1,
        .r_steer_id = 3,
        .f_wheel_id = 2,
        .l_wheel_id = 1,
        .r_wheel_id = 3,
        .f_steer_calib = {.reverse = true, .zero_offset = 3080.f / 8191.f * M_PI * 2.f},
        .l_steer_calib = {.reverse = true, .zero_offset = 7885.f / 8191.f * M_PI * 2.f},
        .r_steer_calib = {.reverse = true, .zero_offset = 1729.f / 8191.f * M_PI * 2.f},
    };
  }
};