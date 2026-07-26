#pragma once

#include <librm.hpp>

#include "config.hpp"
#include "shared_resources.hpp"

struct GimbalResources {
  explicit GimbalResources(const GimbalConfig &config)
      : config{config},  //
        lift_motor(SharedResources::GetInstance().mcp_can, {.master_id = 0xb1,
                                                            .slave_id = 0xa1,
                                                            .p_max = 3.141593f,
                                                            .v_max = 20.f,
                                                            .t_max = 28.f,
                                                            .kp_range = {0, 500},
                                                            .kd_range = {0, 50}}),
        pitch_motor(SharedResources::GetInstance().mcp_can, config.pitch_motor_id) {
    lift_motor.SetName("lift_motor");
    pitch_motor.SetName("pitch_motor");
    device_manager << &lift_motor << &pitch_motor;
  }

  GimbalConfig config;

  rm::device::DmMotorMit lift_motor;
  rm::device::M2006 pitch_motor;
  rm::modules::MotorCalibrator camera_lift_calib;

  rm::modules::TrajectoryLimiter lift_tl{1.f, 1.1f};

  bool current_pitch_command_up{true};

  rm::device::DeviceManager<10> device_manager;  ///< 管理所有电机的在线状态
};