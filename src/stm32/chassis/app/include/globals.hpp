#pragma once

#include <librm.hpp>

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "et16s.hpp"
#include "tri_steer_chassis_controller.hpp"
#include "rc_forward.hpp"
#include "vt_link_forward.hpp"

/**
 * @brief   全局数据仓库
 */
struct Globals {
  rm::hal::ThrottledCan<128> left_can{hfdcan1, 7000}, right_can{hfdcan2, 7000}, comm_can{hfdcan3, 1000};
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kInterrupt};  ///< 接收遥控器的串口
  rm::hal::Serial motor_485_serial{huart2, 100, rm::hal::stm32::UartMode::kInterrupt,
                                   rm::hal::stm32::UartMode::kInterrupt};  ///< 电机485通信串口

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
  rm::modules::MahonyAhrs mahony{2000.f};

  WflyET16s rc{sbus_serial};                                               ///< 遥控器
  RcForwarder rc_forwarder{comm_can};                                      ///< 用于把遥控器数据转发到CAN总线上
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> ref_vt_link;  ///< 图传链路
  rm::device::VirtualDevice custom_ctrl_device, vt_remote_device;
  VtLinkForwarder vt_link_forwarder{comm_can};  ///< 用于把图传链路数据转发到CAN总线上
  rm::modules::KeyboardHandler<> kh{
      rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kCtrl |  //
      rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kShift   //
  };  ///< 键盘处理器，用于处理客户端的键盘输入

  TriSteerChassisController controller;

  struct Actuator {
    rm::device::GM6020 *f_steer, *l_steer, *r_steer;  ///< 舵电机
    rm::device::M3508 *f_wheel, *l_wheel, *r_wheel;   ///< 轮电机
    rm::device::Go8010Motor *camera_lift;             ///< 升降杆电机
    rm::modules::MotorCalibrator                      //
        f_steer_calib{true, 2.62112617f},             //
        l_steer_calib{true, 6.03081465f},             //
        r_steer_calib{true, 1.34316409f};
  } actuator;
  rm::device::DeviceManager<10> device_manager;  ///< 设备管理器

  etl::string<512> system_status;  ///< 系统状态摘要字符串

  float vbus;

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
    motor_485_serial.Begin();

    actuator.f_steer = new rm::device::GM6020(right_can, 1);
    actuator.l_steer = new rm::device::GM6020(right_can, 2);
    actuator.r_steer = new rm::device::GM6020(right_can, 3);
    actuator.f_wheel = new rm::device::M3508(right_can, 2);
    actuator.l_wheel = new rm::device::M3508(right_can, 3);
    actuator.r_wheel = new rm::device::M3508(right_can, 1);
    actuator.camera_lift = new rm::device::Go8010Motor(motor_485_serial, 1);

    rc.SetName("rc");
    actuator.f_steer->SetName("f_steer");
    actuator.l_steer->SetName("l_steer");
    actuator.r_steer->SetName("r_steer");
    actuator.f_wheel->SetName("f_wheel");
    actuator.l_wheel->SetName("l_wheel");
    actuator.r_wheel->SetName("r_wheel");
    actuator.camera_lift->SetName("camera_lift");

    // 把设备都注册给设备管理器
    device_manager << &rc << actuator.f_steer << actuator.l_steer << actuator.r_steer << actuator.f_wheel
                   << actuator.l_wheel << actuator.r_wheel;
    //<< actuator.camera_lift;
  }
};

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
struct GlobalsNoDtcm {
  uint16_t adc_val[2];  ///< ADC采样值，必须在DMA可访问的RAM_D1中（DTCM不可被DMA访问）
  rm::hal::Serial vt_link_serial_7{huart7, 25, rm::hal::stm32::UartMode::kInterrupt,
                                   rm::hal::stm32::UartMode::kInterrupt},                                          //
      vt_link_serial_10{huart10, 25, rm::hal::stm32::UartMode::kInterrupt, rm::hal::stm32::UartMode::kInterrupt};  //
  ///< 图传链路的串口，两路功能完全相同，互做冗余，一路出故障只需要把线接到另一路上即可
};

extern Globals *globals;
extern __attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;
