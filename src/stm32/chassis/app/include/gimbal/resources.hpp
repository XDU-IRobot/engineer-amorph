#pragma once

#include <librm.hpp>

#include "config.hpp"
#include "shared_resources.hpp"
#include "camera_lift_dynamics.hpp"

struct GimbalResources {
  explicit GimbalResources(const GimbalConfig &config)
      : config{config},                                                   //
        imu{SharedResources::GetInstance().mcp_can, config.imu_node_id},  //
        yaw_motor{SharedResources::GetInstance().mcp_can, config.yaw_motor_id},
        lift_motor(SharedResources::GetInstance().mcp_can, {.master_id = 37, .slave_id = 21}),
        yaw_calib{config.yaw_calib} {
    pitch_servo.SetName("pitch_servo");
    imu.SetName("imu");
    yaw_motor.SetName("yaw_motor");
    lift_motor.SetName("lift_motor");
    device_manager << &imu << &yaw_motor << &lift_motor;

    imu_yaw_pid.SetDiffFirst(true).SetDiffLpfAlpha(1.f).SetCircular(true).SetCircularCycle(2.f * M_PI);
    encoder_yaw_pid.SetDiffFirst(true).SetDiffLpfAlpha(0.9f).SetCircular(true).SetCircularCycle(2.f * M_PI);
  }

  GimbalConfig config;

  rm::hal::Pin servo_tx_en{SERVO_TX_EN_GPIO_Port, SERVO_TX_EN_Pin}, servo_rx_en{SERVO_RX_EN_GPIO_Port, SERVO_RX_EN_Pin};
  rm::device::HiWonderServo pitch_servo{SharedResources::GetInstance().no_dtcm->servo_serial,  //
                                        servo_tx_en, servo_rx_en, 0};                          ///< pitch舵机

  rm::device::HipnucImuCan imu;  ///< IMU
  rm::device::GM6020 yaw_motor;  ///< 舵电机
  rm::device::Ht04Motor lift_motor;
  rm::modules::MotorCalibrator yaw_calib, camera_lift_calib;

  rm::modules::TrajectoryLimiter yaw_tl{50.f, 100.f};
  rm::modules::TrajectoryLimiter lift_tl{0.9f, 5.f};
  bool lift_calibrated{false};

  rm::modules::PID imu_yaw_pid{40000.f, 0.f, 0.f, 6200.f, 0.f};
  rm::modules::PID encoder_yaw_pid{40000.f, 0.f, 0.f, 15000.f, 0.f};

  rm::device::DeviceManager<10> device_manager;  ///< 管理所有电机的在线状态
};