#pragma once

#include <librm.hpp>

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "bsp/vbus_adc.hpp"
#include "et16s.hpp"

/**
 * STM32H7的DTCM区域只能被CPU访问，不能被DMA访问，所以需要开一块在DTCM之外的全局数据区域，专门放一些需要被DMA访问的对象
 * 注意linker script里要相应的开一块放在RAM_D1里的区域sram4：
 *   .sram4 (NOLOAD) :
 * {
 *   . = ALIGN(32);
 *   *(.sram4)
 *   . = ALIGN(32);
 * } >RAM_D1
 */
struct SharedResourcesNoDtcm {
  rm::hal::Serial<25> vt_link_serial_7{huart7, false, true}, vt_link_serial_10{huart10, false, true};
  ///< 图传链路的串口，两路功能完全相同，互做冗余，一路出故障只需要把线接到另一路上即可
  rm::hal::Serial<100> servo_serial{huart1, false, true};      ///< 舵机串口
  rm::hal::Serial<50> sbus_serial{huart5, false, true};        ///< 接收遥控器的串口
  rm::hal::Serial<100> motor_485_serial{huart2, false, true};  ///< 电机485通信串口
  VbusAdc vbus_adc{hadc1};
};
extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

/**
 * @brief   板级全局共享资源
 */
struct SharedResources {
  SharedResourcesNoDtcm *no_dtcm{&shared_no_dtcm};  ///< DTCMRAM外的资源指针

  rm::hal::ThrottledCan<128> left_can{4000, hfdcan1}, right_can{4000, hfdcan2}, comm_can{1000, hfdcan3};
  rm::hal::ThrottledMcp2515<128> mcp_can{4000, hspi1, MCP_CS_GPIO_Port, MCP_CS_Pin,
                                         rm::hal::stm32::Mcp2515::Bitrate::k1Mbps};

  rm::hal::Pin servo_tx_en{*SERVO_TX_EN_GPIO_Port, SERVO_TX_EN_Pin},
      servo_rx_en{*SERVO_RX_EN_GPIO_Port, SERVO_RX_EN_Pin};
  rm::device::HiWonderServo servo{shared_no_dtcm.servo_serial, servo_tx_en, servo_rx_en, 0};

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

  rm::device::BMI088 imu{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};

  WflyET16s rc{shared_no_dtcm.sbus_serial};                                ///< 遥控器
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> ref_vt_link;  ///< 图传链路
  rm::device::VirtualDevice custom_ctrl_device, vt_remote_device;
  rm::modules::KeyboardHandler<> kh{
      rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kCtrl |  //
      rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kShift   //
  };  ///< 键盘处理器，用于处理客户端的键盘输入

  etl::string<512> system_status;  ///< 系统状态摘要字符串

  float vbus;

  /**
   * @brief 获取单例实例
   */
  static SharedResources &GetInstance() {
    static SharedResources *shared_resources_instance{nullptr};
    if (shared_resources_instance == nullptr) {
      shared_resources_instance = new SharedResources;
      shared_resources_instance->Init();
    }
    return *shared_resources_instance;
  }

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    buzzer.Begin();
    left_can.SetFilter(0, 0);
    right_can.SetFilter(0, 0);
    comm_can.SetFilter(0, 0);
    left_can.Begin();
    right_can.Begin();
    comm_can.Begin();
    mcp_can.Begin();
    rc.Begin();
    shared_no_dtcm.motor_485_serial.Start();
    shared_no_dtcm.servo_serial.Start();
    shared_no_dtcm.vbus_adc.Begin();
  }
};
