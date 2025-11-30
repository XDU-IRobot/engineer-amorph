#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"

#include "bsp/buzzer.hpp"
#include "fsm.hpp"
#include "bsp/led.hpp"
#include "device_manager.hpp"
#include "rpc/rpc.hpp"

/**
 * @brief   全局数据仓库
 */
struct Globals {
  rm::hal::Can left_can{hfdcan1}, right_can{hfdcan2}, rpc_can{hfdcan3};  ///< 用于电机和RPC通信的CAN总线
  rm::hal::Serial sbus_serial{huart5, 50, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kInterrupt};  ///< 用于接收遥控器信号的串口
  RPC rpc;                                                            ///< RPC对象，负责与上位机通信

  Buzzer buzzer{htim12, TIM_CHANNEL_2};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Startup, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::TheLick,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain>
      buzzer_controller;
  WS2812Led led{hspi6};                                                 ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash,     //
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  rm::device::Sbus rc{sbus_serial};  ///< 遥控器
  fsm::Arm fsm;                      ///< 机械臂的有限状态机
  struct Actuator {
    using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
    using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;
    DmMit *j1, *j2, *j3, *j4, *j5;
    rm::device::M2006 *j6;
  } actuator;

  DeviceManager<10> device_manager;  ///< 设备管理器

  rm::modules::SparseValueWatcher<int> rc_sd_watcher{0};

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    actuator.j1 = new rm::device::DmMotor{
        left_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x20,
                                                                                    .slave_id = 0x21,
                                                                                    .p_max = 3.141593f,
                                                                                    .v_max = 45.f,
                                                                                    .t_max = 54.f,
                                                                                    .kp_range = {0, 500},
                                                                                    .kd_range = {0, 50}}};
    actuator.j2 = new rm::device::DmMotor{
        left_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x22,
                                                                                    .slave_id = 0x23,
                                                                                    .p_max = 3.141593f,
                                                                                    .v_max = 45.f,
                                                                                    .t_max = 54.f,
                                                                                    .kp_range = {0, 500},
                                                                                    .kd_range = {0, 50}}};
    actuator.j3 = new rm::device::DmMotor{
        left_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x24,
                                                                                    .slave_id = 0x25,
                                                                                    .p_max = 3.141593f,
                                                                                    .v_max = 45.f,
                                                                                    .t_max = 54.f,
                                                                                    .kp_range = {0, 500},
                                                                                    .kd_range = {0, 50}}};
    actuator.j4 = new rm::device::DmMotor{
        right_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x26,
                                                                                     .slave_id = 0x27,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    actuator.j5 = new rm::device::DmMotor{
        right_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0x28,
                                                                                     .slave_id = 0x29,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    actuator.j6 = new rm::device::M2006(left_can, 2);

    auto rpc_transport = std::make_shared<FdCanTransport>(rpc_can, 0x23, 0x33);
    rpc.AttachToConnection(rpc_transport);
    buzzer.Init();
    left_can.SetFilter(0, 0);
    right_can.SetFilter(0, 0);
    left_can.Begin();
    right_can.Begin();
    rpc_can.Begin();
    rc.Begin();

    // 把设备都注册给设备管理器
    device_manager << &rc
                   // << actuator.j1
                   << actuator.j2 << actuator.j3 << actuator.j4 << actuator.j5 << actuator.j6;
  }
};

extern Globals *globals;