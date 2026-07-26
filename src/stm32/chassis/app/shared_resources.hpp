#pragma once

#include <librm.hpp>

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "bsp/ahrs.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "bsp/vbus_adc.hpp"
#include "et16s.hpp"
#include "global_config.hpp"
#include "client/broker.hpp"
#include "client/xbox_controller.hpp"
#include "custom_controller.hpp"
#include "client/jetson.hpp"
#include "client/jetson_2point.hpp"
#include "ui/TaskScheduler.hpp"
#include "arm/eu_storage.hpp"

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
  rm::hal::Serial<25> vt_link_serial{huart1, false, true}, pm_link_serial{huart10, false, true};
  ///< 图传链路的串口，两路功能完全相同，互做冗余，一路出故障只需要把线接到另一路上即可
  rm::hal::Serial<100> cli_serial{huart7, false, true};  ///< 舵机串口
  rm::hal::Serial<50> sbus_serial{huart5, false, true};  ///< 接收遥控器的串口
  VbusAdc vbus_adc{hadc1};
};
extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

/**
 * @brief   板级全局共享资源
 */
struct SharedResources {
  SharedResourcesNoDtcm *no_dtcm{&shared_no_dtcm};  ///< DTCMRAM外的资源指针

  rm::hal::ThrottledCan<100> left_can{6500, hfdcan1}, right_can{6500, hfdcan2};
  rm::hal::ThrottledCan<100> comm_can{2500, hfdcan3};
  rm::hal::ThrottledMcp2515<100> mcp_can{2500, hspi1, MCP_CS_GPIO_Port, MCP_CS_Pin,
                                         rm::hal::stm32::Mcp2515::Bitrate::k1Mbps};

  Buzzer buzzer{htim12, TIM_CHANNEL_2};  ///< 蜂鸣器
  rm::modules::BuzzerController<
      rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>, rm::modules::buzzer_melody::Beeps<2>,
      rm::modules::buzzer_melody::Startup, rm::modules::buzzer_melody::Error, rm::modules::buzzer_melody::Success,
      rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kC6>,
      rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kD6>,
      rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kE6>,
      rm::modules::buzzer_melody::Tone<rm::modules::NoteFreqStandard::kF6>, buzzer_melody::EnterManualMode,
      buzzer_melody::HiBeeps<1>, buzzer_melody::HiBeeps<2>, buzzer_melody::HiBeeps<3>, buzzer_melody::HiBeeps<4>,
      buzzer_melody::HiBeeps<5>, buzzer_melody::HiBeeps<6>, rm::modules::buzzer_melody::TheLick>
      buzzer_controller;
  WS2812Led led{hspi6};                                                 ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash,     //
                                led_pattern::YellowFlash,               //
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  Ahrs ahrs{Ahrs::PYPXPZ};

  rm::device::VT03 vt_rc;                                      ///< 图传遥控器
  WflyET16s rc{shared_no_dtcm.sbus_serial};                    ///< 遥控器
  rm::device::Referee<GlobalConfig::kRefereeRev> ref_pm;       ///< 电管模块串口
  rm::device::Referee<GlobalConfig::kRefereeRev> ref_vt_link;  ///< 图传链路
  CustomClientBroker client_broker{ref_vt_link};               ///< 自定义客户端Broker
  XboxController xbox_controller{client_broker};
  CustomController custom_controller{ref_vt_link};
  rm::device::DeviceManager<5> device_manager;  ///< 设备管理器，用于监测各个设备的在线状态
  rm::modules::KeyboardHandler<> kh{rm::device::VT03::kCtrl |
                                    rm::device::VT03::kShift};  ///< 键盘处理器，用于处理客户端的键盘输入
  Jetson vision_jetson{comm_can};
  Jetson2Point jetson{comm_can};

  EnergyUnitStorage eu_storage{{STORAGE_1_GPIO_Port, STORAGE_1_Pin}, {STORAGE_2_GPIO_Port, STORAGE_2_Pin}};

  rm::device::UITaskScheduler ui_ts{28.f};

  etl::string<512> system_status;  ///< 系统状态摘要字符串

  float vbus;

  /**
   * @brief 在指定内存地址构造单例（placement new），必须在首次 GetInstance() 前调用
   * @param addr 对齐后的内存地址，大小至少为 sizeof(SharedResources)
   */
  static void PlaceAt(void *addr) {
    instance_ = new (addr) SharedResources;
    instance_->Init();
  }

  /**
   * @brief 获取单例实例
   * @note PlaceAt() 必须先于本函数调用
   */
  static SharedResources &GetInstance() { return *instance_; }

 private:
  inline static SharedResources *instance_{nullptr};

 public:
  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    rc.SetName("remote");
    vt_rc.SetName("vt_remote");
    client_broker.SetName("client_broker");
    device_manager << &rc << &vt_rc << &client_broker;

    rm::hal::SerialRxCallbackFunction on_vt_serial_rx = [&](etl::span<const uint8_t> data) {
      for (auto byte : data) {
        ref_vt_link << byte;
        vt_rc << byte;
      }
      kh.Update(vt_rc.data().keyboard_key);
    };
    no_dtcm->vt_link_serial.AttachRxCallback(on_vt_serial_rx);  // 注册接收回调，把收到的数据传给解码器

    rm::hal::SerialRxCallbackFunction on_pm_serial_rx = [&](etl::span<const uint8_t> data) {
      for (auto byte : data) {
        ref_pm << byte;
      }
    };
    no_dtcm->pm_link_serial.AttachRxCallback(on_pm_serial_rx);  // 注册接收回调，把收到的数据传给解码器

    device_manager.OnDeviceStatusChange([&](rm::device::Device *dev) {
      if (dev->name() == "remote") {
        if (dev->online_status() == rm::device::Device::kOk) {
          buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
          led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
        } else if (dev->online_status() != rm::device::Device::kOk) {
          buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
          led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
        }
      } else if (dev->name() == "client_broker") {
        client_broker.ResetData();
      }
    });

    buzzer.Begin();
    left_can.SetFilter(0, 0);
    right_can.SetFilter(0, 0);
    comm_can.SetFilter(0, 0);
    left_can.Begin();
    right_can.Begin();
    comm_can.Begin();
    mcp_can.Begin();
    rc.Begin();
    no_dtcm->vt_link_serial.Start();
    no_dtcm->pm_link_serial.Start();
    no_dtcm->cli_serial.Start();
    no_dtcm->vbus_adc.Begin();
  }
};
