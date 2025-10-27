
#pragma once

#include <librm.hpp>

#include "spi.h"
#include "usart.h"
#include "can.h"
#include "dynamics.hpp"
#include "rgb_led.hpp"

struct Globals {
  rm::device::BMI088 *imu;
  rm::modules::MahonyAhrs ahrs;

  rm::hal::Serial *dbus;
  rm::device::DR16 *rc;

  rm::hal::Can *motor_can;
  DynamicModel dynamic_model;
  LED led;

  struct {
    rm::device::LkMotor<rm::device::LkMotorType::kMS, 15> *j0, *j3, *j4, *j5;
    rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *j1, *j2;
  } motors;

  void Init() {
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
    rc = new rm::device::DR16{*dbus};
    motor_can = new rm::hal::Can{hcan1};
    motors.j0 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{*motor_can, 1};
    motors.j1 = new rm::device::DmMotor{
        *motor_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x25,
                                                                                      .slave_id = 0x15,
                                                                                      .p_max = 3.141593f,
                                                                                      .v_max = 30.f,
                                                                                      .t_max = 10.f,
                                                                                      .kp_range = {0, 500},
                                                                                      .kd_range = {0, 50}}};
    motors.j2 =
        new rm::device::DmMotor{*motor_can,
                                rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x26,
                                                                                                  .slave_id = 0x16,
                                                                                                  .p_max = 3.141593f,
                                                                                                  .v_max = 30.f,
                                                                                                  .t_max = 10.f,
                                                                                                  .kp_range = {0, 500},
                                                                                                  .kd_range = {0, 50}},
                                true};
    motors.j3 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{*motor_can, 1};
    motors.j4 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{*motor_can, 1};
    motors.j5 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{*motor_can, 1};
    rc->Begin();
    motor_can->SetFilter(0, 0);
    motor_can->Begin();
    led.Init();
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);  // buzzer
  };
};

extern Globals globals;