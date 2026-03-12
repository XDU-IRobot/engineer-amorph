
#pragma once

#include <librm.hpp>

#include <etl/debounce.h>

#include "usart.h"
#include "can.h"
#include "dynamics.hpp"
#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "encoder_counter.hpp"

struct Globals {
  rm::modules::MahonyAhrs ahrs;  //

  // rm::hal::Serial dbus{huart1, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};  //
  // rm::device::DR16 rc{dbus};  ///< 遥控器，调试用的，自定义控制器就别给他整个遥控器了

  rm::hal::Can motor_can{hcan1};
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
    rm::device::LkMotor<rm::device::LkMotorType::kMS, 15> *j0, *j3, *j4;
    rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *j1, *j2;
    std::array<rm::modules::MotorCalibrator, 6> calibrators{
        rm::modules::MotorCalibrator{false, 1.84754455f},  //
        rm::modules::MotorCalibrator{false, 0.f},          //
        rm::modules::MotorCalibrator{true, 0.f},           //
        rm::modules::MotorCalibrator{true, 0.f},           //
        rm::modules::MotorCalibrator{true, 0.221666992f},  //
        rm::modules::MotorCalibrator{true, 0.f},           //
    };
    EncoderCounter j5_encoder{M_PI * 2.f};
    rm::modules::ModeFilter<float, 5> j5_encoder_filter{1};

    std::array<float, 6> GetPositions() {
      std::array<float, 6> positions{
          j0->feedback().position_rad,                         //
          j1->pos(),                                           //
          j2->pos(),                                           //
          j3->feedback().position_rad,                         //
          j4->feedback().position_rad,                         //
          j5_encoder_filter.apply(j5_encoder.RealPosition()),  //
      };
      // 对每个电机应用反转和零点偏移补偿
      for (size_t i = 0; i < 6; ++i) {
        positions[i] = calibrators[i].RealToLogical(positions[i]);
      }
      // 3个瓴控的电机编码器是0~2pi，需要处理到-pi~pi
      positions[0] = rm::modules::Wrap(positions[0], -M_PI, M_PI);
      positions[3] = rm::modules::Wrap(positions[3], -M_PI, M_PI);
      positions[4] = rm::modules::Wrap(positions[4], -M_PI, M_PI);
      return positions;
    }
  } motors;

  struct {
    etl::debounce<10, 10, 10> a, b, c, d, e;
  } buttons;

  void Init() {
    motors.j0 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{motor_can, 1};
    motors.j1 = new rm::device::DmMotor{
        motor_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x25,
                                                                                     .slave_id = 0x15,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    motors.j2 = new rm::device::DmMotor{
        motor_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x26,
                                                                                     .slave_id = 0x16,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    motors.j3 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{motor_can, 2};
    motors.j4 = new rm::device::LkMotor<rm::device::LkMotorType::kMS, 15>{motor_can, 3};
    // rc.Begin();
    motor_can.SetFilter(0, 0);
    motor_can.Begin();
    led.Init();
    led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    buzzer.Init();
  };
};

extern Globals *globals;