
#include <etl/cyclic_value.h>

#include "fsm.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};
__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->rc.Update();
  globals->system_status = globals->device_manager.GetSummaryString();

  static etl::cyclic_value<int, 0, 499> loop_divisor;
  ++loop_divisor;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
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

  const fsm::event::ControlLoop control_loop_event = [] {
    using rm::modules::Wrap;
    fsm::event::ControlLoop ret;
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
    return ret;
  }();
  fsm::chassis.receive(control_loop_event);
}
}  // namespace

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();  // 初始化全局对象
  fsm::Init();      // 初始化状态机
  InitUserInput();  // 初始化用户输入处理

  // 监听设备状态变化，所有设备正常时LED为绿色呼吸灯，否则红灯闪烁
  globals->device_manager.OnDeviceStatusChange([](rm::device::Device *) {
    if (globals->device_manager.all_device_ok()) {
      globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    } else {
      globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    }
  });

  // 初始化图传链路串口
  rm::hal::SerialRxCallbackFunction on_vt_serial_rx = [&](const std::vector<uint8_t> &data, uint16_t len) {
    for (int i = 0; i < len; ++i) {
      globals->ref_vt_link << data[i];
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
        // 从图传链路解包出遥控器数据，转发到CAN总线上
        globals->vt_remote_device.ReportStatus(rm::device::Device::kOk);
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
  globals_no_dtcm.vt_link_serial_7.Begin();
  // globals_no_dtcm.vt_link_serial_10.Begin();

  // 创建主循环定时任务，定频500hz
  TimerTask mainloop{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop.Start();

  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  for (;;) {
  }
}
}
