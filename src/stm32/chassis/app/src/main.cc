
#include <etl/cyclic_value.h>

#include "fsm.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->rc.Update();

  static etl::cyclic_value<int, 0, 499> loop_divisor;
  ++loop_divisor;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }

  const fsm::event::ControlLoop control_loop = [] {
    fsm::event::ControlLoop ret;
    ret.f_steer_pos = globals->actuator.f_steer->pos_rad();
    ret.f_steer_vel = globals->actuator.f_steer->rpm() / 60.f * 2.f * M_PI;  // to rad/s
    ret.l_steer_pos = globals->actuator.l_steer->pos_rad();
    ret.l_steer_vel = globals->actuator.l_steer->rpm() / 60.f * 2.f * M_PI;
    ret.r_steer_pos = globals->actuator.r_steer->pos_rad();
    ret.r_steer_vel = globals->actuator.r_steer->rpm() / 60.f * 2.f * M_PI;
    ret.f_wheel_vel = globals->actuator.f_wheel->rpm() / 60.f * 2.f * M_PI;
    ret.l_wheel_vel = globals->actuator.l_wheel->rpm() / 60.f * 2.f * M_PI;
    ret.r_wheel_vel = globals->actuator.r_wheel->rpm() / 60.f * 2.f * M_PI;
    return ret;
  }();
  fsm::chassis.receive(control_loop);
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

  // 注册图传链路串口接收回调函数
  rm::hal::SerialRxCallbackFunction vt_link_rx_callback = [](const std::vector<uint8_t> &data, uint16_t length) {
    for (uint16_t i = 0; i < length; ++i) {
      globals->ref_vt_link << data[i];
    }
  };
  globals->vt_link_serial.AttachRxCallback(vt_link_rx_callback);
  globals->vt_link_serial.Begin();  // 开始接收图传链路数据

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
