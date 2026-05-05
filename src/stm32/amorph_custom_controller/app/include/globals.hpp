#pragma once

#include <librm.hpp>

#include "can.h"

#include "usart.h"
#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "buttons.hpp"
#include "actuator.hpp"

struct Globals {
  rm::modules::MahonyAhrs ahrs;  //

  // rm::hal::Serial dbus{huart1, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};  //
  // rm::device::DR16 rc{dbus};  ///< 遥控器，调试用的，自定义控制器就别给他整个遥控器了
  rm::hal::ThrottledCan<128> motor_can{3000, hcan1};
  rm::hal::Serial<128> servo_serial{huart2, false, false};

  rm::hal::Pin g_pin{LED_G_GPIO_Port, LED_G_Pin}, r_pin{LED_R_GPIO_Port, LED_R_Pin};
  rm::device::HiWonderServo servo{servo_serial, g_pin, r_pin, 254};

  LED led;
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;
  Buzzer buzzer;
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Startup, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Beeps<2>,
                                rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kC6>>
      buzzer_controller;

  Actuator actuator{motor_can};
  Buttons buttons;

  void Init() {
    // rc.Begin();
    motor_can.SetFilter(0, 0);
    motor_can.Begin();
    led.Init();
    led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    buzzer.Init();
  };
};

extern Globals *globals;