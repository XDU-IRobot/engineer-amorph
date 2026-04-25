
#include <etl/cyclic_value.h>

#include "adc.h"

#include "../include/chassis/fsm_chassis.hpp"
#include "fsm_gimbal.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};
__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;

volatile long long timestamp_us = 0;
volatile long long mainloop_time_us = 0;

namespace {
void SubLoop500hz() {
  timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const auto start = std::chrono::steady_clock::now();

  globals->device_manager.Update();
  globals->rc.Update();
  globals->system_status = globals->device_manager.GetSummaryString();
  globals->vbus = globals_no_dtcm.vbus_adc.vbus();

  static etl::cyclic_value<int, 0, 499> loop_divisor;
  ++loop_divisor;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    // globals->servo.MoveTime((HAL_GetTick() % 5000) / 5, 20);
    globals->servo.ReadTemperature();
    // 转发遥控器数据
    if (globals->rc.online_status() == rm::device::Device::Status::kOk) {
      int16_t rc_channels[16];
      for (int i = 0; i < 16; ++i) {
        rc_channels[i] = globals->rc.channel(i);
      }
      globals->rc_forwarder.Write(rc_channels);
    }

    // 更新LED和蜂鸣器
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }

  const fsm_chassis::event::ControlLoop control_loop_event = [] {
    using rm::modules::Wrap;
    fsm_chassis::event::ControlLoop ret;
    ret.f_steer_pos =
        Wrap(globals->actuator.f_steer_calib.RealToLogical(globals->actuator.f_steer->pos_rad()), 0, M_PI * 2.f);
    ret.f_steer_vel = -globals->actuator.f_steer->rpm() / 60.f * 2.f * M_PI;  // to rad/s
    ret.l_steer_pos =
        Wrap(globals->actuator.l_steer_calib.RealToLogical(globals->actuator.l_steer->pos_rad()), 0, M_PI * 2.f);
    ret.l_steer_vel = -globals->actuator.l_steer->rpm() / 60.f * 2.f * M_PI;
    ret.r_steer_pos =
        Wrap(globals->actuator.r_steer_calib.RealToLogical(globals->actuator.r_steer->pos_rad()), 0, M_PI * 2.f);
    ret.r_steer_vel = -globals->actuator.r_steer->rpm() / 60.f * 2.f * M_PI;
    ret.f_wheel_vel = -globals->actuator.f_wheel->rpm() / 60.f * 2.f * M_PI;
    ret.l_wheel_vel = -globals->actuator.l_wheel->rpm() / 60.f * 2.f * M_PI;
    ret.r_wheel_vel = -globals->actuator.r_wheel->rpm() / 60.f * 2.f * M_PI;
    ret.f_wheel_current = -globals->actuator.f_wheel->current();
    ret.l_wheel_current = -globals->actuator.l_wheel->current();
    ret.r_wheel_current = -globals->actuator.r_wheel->current();
    return ret;
  }();
  fsm_chassis::chassis.receive(control_loop_event);
  fsm_gimbal::gimbal.receive(fsm_gimbal::event::ControlLoop{});

  mainloop_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}

void AhrsLoop2khz() {
  globals->imu.Update();
  globals->mahony.Update(                               //
      rm::modules::ImuData6Dof{globals->imu.gyro_x(),   //
                               globals->imu.gyro_y(),   //
                               globals->imu.gyro_z(),   //
                               globals->imu.accel_x(),  //
                               globals->imu.accel_y(),  //
                               globals->imu.accel_z()});
}

// 2khz
void MainLoop2khz() {
  static size_t loop_divisor = 0;
  loop_divisor = (loop_divisor + 1) % 2000;

  AhrsLoop2khz();
  if (loop_divisor % 4 == 0) {  // 500hz
    SubLoop500hz();
  }
}

}  // namespace

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MCP_INT_Pin) {
    globals->mcp_can.ExtiIrqHandler();
  }
}

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();      // 初始化全局对象
  fsm_chassis::Init();  //
  fsm_gimbal::Init();   // 初始化状态机

  // 定义遥控器拨杆功能
  globals->rc(rc_ch::SD).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // SD在下
      fsm_chassis::chassis.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // SD在上
      fsm_chassis::chassis.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNormal});
    }
    if (new_value > 500 && new_value < 1100) {  // SD在中
      fsm_chassis::chassis.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
    }
  });
  // 定义遥控器拨杆功能
  globals->rc(rc_ch::SA).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // SD在下
      fsm_gimbal::gimbal.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // SD在上
      fsm_gimbal::gimbal.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kNormal});
    }
    if (new_value > 500 && new_value < 1100) {  // SD在中
      fsm_gimbal::gimbal.receive(fsm_gimbal::event::ForceModeSwitch{fsm_gimbal::StateId::kManual});
    }
  });

  // 定义键盘键位
  globals->kh
      .OnKeyDown(rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kF,
                 [](const rm::modules::KeyEvent &e) {
                   // 杆升
                   fsm_gimbal::gimbal.receive(fsm_gimbal::event::KeyboardCommand{true});
                 })
      .OnKeyDown(rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kC,
                 [](const rm::modules::KeyEvent &e) {
                   // 杆降
                   fsm_gimbal::gimbal.receive(fsm_gimbal::event::KeyboardCommand{false});
                 });

  // 监听设备状态变化，所有设备正常时LED为绿色呼吸灯，否则红灯闪烁
  globals->device_manager.OnDeviceStatusChange([](rm::device::Device *) {
    if (globals->device_manager.all_device_ok()) {
      globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    } else {
      globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    }
  });

  // 初始化图传链路串口
  rm::hal::SerialRxCallbackFunction on_vt_serial_rx = [&](etl::span<const uint8_t> data) {
    for (auto byte : data) {
      globals->ref_vt_link << byte;
    }
  };
  globals->ref_vt_link.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
    UNUSED(seq);
    switch (cmd_id) {
      case rm::device::RefereeCmdId<rm::device::RefereeRevision::kNewV110>::kCustomRobotData: {
        // 从图传链路解包出自定义控制器数据，转发到CAN总线上
        globals->custom_ctrl_device.ReportStatus(rm::device::Device::kOk);
        const auto &data = globals->ref_vt_link.data().custom_robot_data.data;
        globals->vt_link_forwarder.WriteCustomCtrl(data, sizeof(data));
        break;
      }
      case rm::device::RefereeCmdId<rm::device::RefereeRevision::kNewV110>::kRemoteControl: {
        globals->vt_remote_device.ReportStatus(rm::device::Device::kOk);
        globals->kh.Update(globals->ref_vt_link.data().remote_control.keyboard_value);
        // 遥控器、键鼠数据转发到CAN总线上
        globals->vt_link_forwarder.Write(globals->ref_vt_link.data().remote_control);
        break;
      }
      default: {
        break;
      }
    }
  });                                                                  // 注册解包回调
  globals_no_dtcm.vt_link_serial_7.AttachRxCallback(on_vt_serial_rx);  // 注册接收回调，把收到的数据传给解码器
  // globals_no_dtcm.vt_link_serial_10.AttachRxCallback(on_vt_serial_rx);
  globals_no_dtcm.vt_link_serial_7.Start();
  // globals_no_dtcm.vt_link_serial_10.Start();

  // 创建主循环定时任务，定频2khz
  TimerTask mainloop{
      &htim13,                                       //
      etl::delegate<void()>::create<MainLoop2khz>()  //
  };
  mainloop.Start();

  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  for (;;) {
    globals->left_can.Process();
    globals->right_can.Process();
    globals->comm_can.Process();
    globals->mcp_can.Process();
    // 兜底轮询：若 MCU 错过了下降沿，INT 引脚仍会持续拉低。
    // 此时临时屏蔽 EXTI 中断（防止与 ISR 重入），检查引脚并补调一次，
    // 再立即重新使能，保证正常路径的实时性不受影响。
    if (HAL_GPIO_ReadPin(MCP_INT_GPIO_Port, MCP_INT_Pin) == GPIO_PIN_RESET) {
      HAL_NVIC_DisableIRQ(MCP_INT_EXTI_IRQn);
      if (HAL_GPIO_ReadPin(MCP_INT_GPIO_Port, MCP_INT_Pin) == GPIO_PIN_RESET) {
        globals->mcp_can.ExtiIrqHandler();
      }
      HAL_NVIC_EnableIRQ(MCP_INT_EXTI_IRQn);
    }
  }
}
}
