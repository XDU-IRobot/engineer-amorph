#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"

#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "et16s.hpp"
#include "tri_steer_chassis_controller.hpp"

/**
 * @brief   全局数据仓库
 */
struct Globals {
  rm::hal::Can left_can{hfdcan1}, right_can{hfdcan2}, comm_can{hfdcan3};
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kInterrupt};  ///< 接收遥控器的串口
  rm::hal::Serial vt_link_serial{huart1, 25, rm::hal::stm32::UartMode::kNormal,
                                 rm::hal::stm32::UartMode::kInterrupt};  ///< 图传链路的串口

  Buzzer buzzer{htim12, TIM_CHANNEL_2};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Beeps<2>, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Error, rm::modules::buzzer_melody::Success,
                                rm::modules::buzzer_melody::TheLick, buzzer_melody::EnterManualMode,
                                buzzer_melody::HiBeeps<1>, buzzer_melody::HiBeeps<2>, buzzer_melody::HiBeeps<3>,
                                buzzer_melody::HiBeeps<4>, buzzer_melody::HiBeeps<5>, buzzer_melody::HiBeeps<6>>
      buzzer_controller;
  WS2812Led led{hspi6};                                                 ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash,     //
                                led_pattern::YellowFlash,               //
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  WflyET16s rc{sbus_serial};                                               ///< 遥控器
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> ref_vt_link;  ///< 图传链路

  TriSteerChassisController controller;

  struct Actuator {
    rm::device::GM6020 *f_steer, *l_steer, *r_steer;  ///< 舵电机
    rm::device::M3508 *f_wheel, *l_wheel, *r_wheel;   ///< 轮电机
    rm::device::Go8010Motor *camera_lift;             ///< 升降杆电机
  } actuator;
  rm::device::DeviceManager<10> device_manager;  ///< 设备管理器

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    buzzer.Init();
    left_can.SetFilter(0, 0);
    right_can.SetFilter(0, 0);
    left_can.Begin();
    right_can.Begin();
    comm_can.Begin();
    rc.Begin();

    // 把设备都注册给设备管理器
    device_manager << &rc << actuator.f_steer << actuator.l_steer << actuator.r_steer << actuator.f_wheel
                   << actuator.l_wheel << actuator.r_wheel << actuator.camera_lift;
  }
};

extern Globals *globals;