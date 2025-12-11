#pragma once

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"

#include "motion_planner.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/led.hpp"
#include "rpc/rpc.hpp"
#include "et16s.hpp"

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
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  WflyET16s rc{sbus_serial};  ///< 遥控器

  MotionPlanner planner;  ///< 运动规划器

  struct Actuator {
    using DmMit = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
    using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;
    void SetEnable(bool enable, int repeats = 5) const {
      const auto cmd = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
      for (int i = 0; i < repeats; ++i) {
        // j1->SendInstruction(cmd);
        j2->SendInstruction(cmd);
        j3->SendInstruction(cmd);
        j4->SendInstruction(cmd);
        j5->SendInstruction(cmd);
        gripper->SendInstruction(cmd);
        if (!enable) {
          j6->SetCurrent(0);
        }
      }
    }
    void ApplyOutput(const MotionPlanner::Output<6> &output, bool apply_position = false) const {
      // j1->SetPosition(0, 0, output.efforts[0], 0, 0);
      if (apply_position) {
        j2->SetPosition(output.positions[1], 0, output.efforts[1], 70.f, 30.f);
        j3->SetPosition(output.positions[1] + output.positions[2], 0, output.efforts[2], 40.f, 15.f);
        j4->SetPosition(0, 0, output.efforts[3], 0, 0);
        j5->SetPosition(0, 0, output.efforts[4], 0, 0);
      } else {
        j2->SetPosition(0, 0, output.efforts[1], 0, 0);
        j3->SetPosition(0, 0, output.efforts[2], 0, 0);
        j4->SetPosition(0, 0, output.efforts[3], 0, 0);
        j5->SetPosition(0, 0, output.efforts[4], 0, 0);
      }
      j6->SetCurrent(output.efforts[5]);
      rm::device::DjiMotor<>::SendCommand();
    }
    DmMit *j1, *j2, *j3, *j4, *j5, *gripper;
    rm::device::M2006 *j6;
  } actuator;
  rm::device::DeviceManager<10> device_manager;  ///< 设备管理器

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    actuator.j1 = new rm::device::DmMotor{
        left_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb1,
                                                                                    .slave_id = 0xa1,
                                                                                    .p_max = 3.141593f,
                                                                                    .v_max = 45.f,
                                                                                    .t_max = 54.f,
                                                                                    .kp_range = {0, 500},
                                                                                    .kd_range = {0, 50}}};
    actuator.j3 = new rm::device::DmMotor{
        left_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb2,
                                                                                    .slave_id = 0xa2,
                                                                                    .p_max = 3.141593f,
                                                                                    .v_max = 45.f,
                                                                                    .t_max = 54.f,
                                                                                    .kp_range = {0, 500},
                                                                                    .kd_range = {0, 50}}};
    actuator.j2 =
        new rm::device::DmMotor{left_can,
                                rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb3,
                                                                                                  .slave_id = 0xa3,
                                                                                                  .p_max = 3.141593f,
                                                                                                  .v_max = 45.f,
                                                                                                  .t_max = 54.f,
                                                                                                  .kp_range = {0, 500},
                                                                                                  .kd_range = {0, 50}},
                                true};
    actuator.j4 = new rm::device::DmMotor{
        right_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb4,
                                                                                     .slave_id = 0xa4,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    actuator.j5 = new rm::device::DmMotor{
        right_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb5,
                                                                                     .slave_id = 0xa5,
                                                                                     .p_max = 3.141593f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};
    actuator.j6 = new rm::device::M2006(left_can, 2);
    actuator.gripper = new rm::device::DmMotor{
        right_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{.master_id = 0xb7,
                                                                                     .slave_id = 0xa7,
                                                                                     .p_max = 12.5f,
                                                                                     .v_max = 30.f,
                                                                                     .t_max = 10.f,
                                                                                     .kp_range = {0, 500},
                                                                                     .kd_range = {0, 50}}};

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
                   << actuator.j2 << actuator.j3 << actuator.j4 << actuator.j5 << actuator.j6 << actuator.gripper;
  }
};

extern Globals *globals;