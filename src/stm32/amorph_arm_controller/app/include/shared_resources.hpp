/**
 * @file  shared_resources.hpp
 * @brief 多臂共享的硬件资源（IMU、RC、蜂鸣器、LED、通信 CAN 总线等）
 *
 * SharedResources 拥有整个系统中唯一的物理外设对象。
 * 多个 ArmInstance 持有指向 SharedResources 的裸指针，借用（而非拥有）这些资源。
 */

#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "rc_forward.hpp"
#include "vt_link_forward.hpp"

/**
 * @brief 全系统唯一的共享硬件资源仓库
 */
struct SharedResources {
  // ---- CAN 总线（所有 ArmInstance 可借用其中某几条） ----
  rm::hal::ThrottledCan<128> fdcan1_bus{hfdcan1, 2000};  ///< FDCAN1 总线
  rm::hal::ThrottledCan<128> fdcan2_bus{hfdcan2, 2000};  ///< FDCAN2 总线
  rm::hal::ThrottledCan<128> comm_can{hfdcan3, 1000};    ///< 通信总线（FDCAN3，与上位机/底盘通信）

  // ---- 传感器 ----
  rm::device::BMI088 imu{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};

  // ---- 串口 ----
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kInterrupt};

  // ---- 遥控器 & 图传链路 ----
  RcReceiver rc{comm_can};                    ///< 遥控器数据接收端
  VtLinkReceiver vt_link_receiver{comm_can};  ///< 图传链路自定义控制器数据接收端

  // ---- 蜂鸣器 ----
  Buzzer buzzer{htim12, TIM_CHANNEL_2};
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Beeps<2>, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Error, rm::modules::buzzer_melody::Success,
                                rm::modules::buzzer_melody::TheLick, buzzer_melody::EnterManualMode,
                                buzzer_melody::HiBeeps<1>, buzzer_melody::HiBeeps<2>, buzzer_melody::HiBeeps<3>,
                                buzzer_melody::HiBeeps<4>, buzzer_melody::HiBeeps<5>, buzzer_melody::HiBeeps<6>>
      buzzer_controller;

  // ---- LED ----
  WS2812Led led{hspi6};
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::GreenBreath,
                                rm::modules::led_pattern::RedFlash, led_pattern::YellowFlash,
                                rm::modules::led_pattern::RgbFlow>
      led_controller;

  /**
   * @brief 初始化共享资源（蜂鸣器、通信 CAN）
   * @note  各 ArmInstance 的电机 CAN 总线由 ArmInstance::Init 自行初始化
   */
  void Init() {
    buzzer.Init();
    fdcan1_bus.SetFilter(0, 0);
    fdcan1_bus.Begin();
    fdcan2_bus.SetFilter(0, 0);
    fdcan2_bus.Begin();
    comm_can.SetFilter(0, 0);
    comm_can.Begin();
  }
};
