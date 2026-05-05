#pragma once

#include <librm.hpp>

#include "config.hpp"
#include "shared_resources.hpp"

struct GimbalResources {
  explicit GimbalResources(const GimbalConfig &config)
      : config{config},                                                   //
        imu{SharedResources::GetInstance().mcp_can, config.imu_node_id},  //
        yaw_motor{SharedResources::GetInstance().mcp_can, config.yaw_motor_id},
        yaw_calib{config.yaw_calib} {
    pitch_servo.SetName("pitch_servo");
    imu.SetName("imu");
    yaw_motor.SetName("yaw_motor");
    device_manager << &pitch_servo << &imu << &yaw_motor;

    imu_yaw_pid.SetDiffFirst(true).SetDiffLpfAlpha(1.f).SetCircular(true).SetCircularCycle(2.f * M_PI);
    encoder_yaw_pid.SetDiffFirst(true).SetDiffLpfAlpha(0.9f).SetCircular(true).SetCircularCycle(2.f * M_PI);
  }

  GimbalConfig config;

  rm::hal::Pin servo_tx_en{SERVO_TX_EN_GPIO_Port, SERVO_TX_EN_Pin}, servo_rx_en{SERVO_RX_EN_GPIO_Port, SERVO_RX_EN_Pin};
  rm::device::HiWonderServo pitch_servo{SharedResources::GetInstance().no_dtcm->servo_serial,  //
                                        servo_tx_en, servo_rx_en, 0};                          ///< pitch舵机

  rm::device::HipnucImuCan imu;            ///< IMU
  rm::device::GM6020 yaw_motor;            ///< 舵电机
  rm::modules::MotorCalibrator yaw_calib;  ///< yaw电机校准器

  rm::modules::PID imu_yaw_pid{47000.f, 0.f, 700.f, 6200.f, 0.f};
  rm::modules::PID encoder_yaw_pid{40000.f, 0.f, 0.f, 15000.f, 0.f};

  rm::device::DeviceManager<10> device_manager;  ///< 管理所有电机的在线状态
};