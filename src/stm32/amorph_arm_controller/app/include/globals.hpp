#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "actuators.hpp"
#include "arm_controller.hpp"
#include "rc_forward.hpp"
#include "vt_link_forward.hpp"
#include "host_comm.hpp"
#include "config.hpp"

/**
 * @brief   全局数据仓库
 */
struct Globals {
  rm::hal::ThrottledCan<128> lower_motor_can{CONFIG_LOWER_MOTOR_CAN, 7000};
  rm::hal::ThrottledCan<128> upper_motor_can{CONFIG_UPPER_MOTOR_CAN, 7000};
  rm::hal::ThrottledCan<128> comm_can{hfdcan3, 1000};    ///< 用于和上位机通信的CAN总线
  Host host{comm_can, CONFIG_HOST_COMM_ID};              ///< 上位机对象
  rm::device::DeviceManager<1> host_connection_watcher;  ///< 监视上位机连接状态的设备管理器
  rm::device::BMI088 imu{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};

  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kInterrupt};  ///< 用于接收遥控器信号的串口
  RcReceiver rc{comm_can};                                            ///< 遥控器

  VtLinkReceiver vt_link_receiver{comm_can};

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

  ArmController<7> arm_controller{CONFIG_JOINT_LIMITS, CONFIG_TRAJECTORY_LIMITS};  ///< 机械臂控制器
  Actuator actuator{lower_motor_can, upper_motor_can};                             ///< 所有电机
  rm::device::DeviceManager<10> device_manager;                                    ///< 设备管理器
  etl::string<512> status_str{""};                                                 ///< 描述当前系统状态的字符串

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    // 给设备起一下名字方便看哪个离线
    rc.SetName("rc");
    actuator.j1.SetName("motor_j1");
    actuator.j2.SetName("motor_j2");
    actuator.j3.SetName("motor_j3");
    actuator.j4.SetName("motor_j4");
    actuator.j5.SetName("motor_j5");
    actuator.j6.SetName("motor_j6");
    actuator.gripper.SetName("motor_gripper");

    // 注册设备状态变化事件回调
    device_manager.OnDeviceStatusChange(
        [&](rm::device::Device *device) { status_str = device_manager.GetSummaryString(); });
    host_connection_watcher.OnDeviceStatusChange([&](rm::device::Device *device) {
      if (host_connection_watcher.all_device_ok()) {
        buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
      } else {
        buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
      }
    });

    // 把设备都注册给设备管理器
    device_manager << &rc << &actuator.j1 << &actuator.j2 << &actuator.j3 << &actuator.j4 << &actuator.j5
                   << &actuator.j6 << &actuator.gripper;
    host_connection_watcher << &host;

    buzzer.Init();
    lower_motor_can.SetFilter(0, 0);
    upper_motor_can.SetFilter(0, 0);
    lower_motor_can.Begin();
    upper_motor_can.Begin();
    comm_can.Begin();
  }
};

extern Globals *globals;