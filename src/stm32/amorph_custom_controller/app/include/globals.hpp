
#pragma once

#include <librm.hpp>

#include "spi.h"
#include "usart.h"
#include "can.h"
#include "dynamics.hpp"
#include "rgb_led.hpp"
#include "buzzer.hpp"

struct Globals {
  rm::device::BMI088 *imu;       ///< 修正基座倾斜用的IMU，暂时用不上
  rm::modules::MahonyAhrs ahrs;  //

  rm::hal::Serial *dbus;  //
  rm::device::DR16 *rc;   ///< 遥控器，调试用的，自定义控制器就别给他整个遥控器了

  rm::hal::Can *motor_can;
  DynamicModel dynamic_model;

  LED led;
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;
  Buzzer buzzer;
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Startup, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::TheLick,
                                rm::modules::buzzer_melody::SuperMario>
      buzzer_controller;

  rm::modules::SparseValueWatcher<bool> g_comp_enable_watcher;

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
    led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    buzzer.Init();
    buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  };
};

extern Globals globals;