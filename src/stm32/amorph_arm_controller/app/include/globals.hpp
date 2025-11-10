#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"

#include "arm_controller.hpp"
#include "buzzer.hpp"
#include "fsm.hpp"
#include "led.hpp"
#include "dynamics.hpp"
#include "device_manager.hpp"
#include "rpc/rpc.hpp"

/**
 * @brief   全局数据仓库
 * @note    能封装的尽量封装、能依赖注入的尽量依赖注入，这里出现的全局变量越少越好，保持数据流向清晰
 */
struct Globals {
  rm::hal::Can motor_can1{hfdcan1}, motor_can2{hfdcan2}, rpc_can{hfdcan3};  ///< 用于电机和RPC通信的CAN总线
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kInterrupt};  ///< 用于接收遥控器信号的串口
  RPC rpc;                                                            ///< RPC对象，负责与上位机通信
  AsyncBuzzer buzzer{htim12, TIM_CHANNEL_2, htim14};                  ///< 蜂鸣器

  WS2812Led led{hspi6};                                                 ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash,     //
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  rm::device::Sbus rc{sbus_serial};  ///< 遥控器
  fsm::Arm fsm;                      ///< 机械臂的有限状态机
  ArmController arm_controller;      ///< 机械臂控制器
  struct Actuator {
    using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
    using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;
    PIDMotorController<DmMit> *joint0, *joint1, *joint2;
  } actuator;

  DeviceManager<10> device_manager;  ///< 设备管理器

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    auto fdcan_transport = std::make_shared<FdCanTransport>(rpc_can, 0x23, 0x33);
    rpc.AttachToConnection(fdcan_transport);
    buzzer.Init();
    motor_can1.Begin();
    motor_can2.Begin();
    rpc_can.Begin();
    // rc.Begin();

    // 把设备都注册给设备管理器
    device_manager << &rc;
  }
};

extern Globals *globals;