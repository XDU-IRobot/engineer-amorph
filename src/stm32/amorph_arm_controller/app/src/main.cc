
#include <etl/cyclic_value.h>

#include <numbers>

#include "fsm.hpp"
#include "rpc/host_api.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->rc.Update();
  const std::array<float, 6> current_positions = {globals->actuator.j1->pos(),                                //
                                                  globals->actuator.j2->pos(),                                //
                                                  globals->actuator.j3->pos() - globals->actuator.j2->pos(),  //
                                                  globals->actuator.j4->pos(),                                //
                                                  globals->actuator.j5->pos(),                                //
                                                  globals->actuator.j6_encoder.RealPosition()};
  const std::array<float, 6> current_velocities = {
      globals->actuator.j1->vel(),                                          //
      globals->actuator.j2->vel(),                                          //
      globals->actuator.j3->vel() - globals->actuator.j2->vel(),            //
      globals->actuator.j4->vel(),                                          //
      globals->actuator.j5->vel(),                                          //
      globals->actuator.j6->rpm() / 60.f * 2.f * std::numbers::pi_v<float>  //
          / 36.f / 1.75f                                                    // 2006减速比1:36，齿轮传动1:1.75
  };
  fsm::arm.receive(fsm::event::ControlLoop{current_positions, current_velocities});

  static etl::cyclic_value<int, 0, 499> loop_divisor;
  ++loop_divisor;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    globals->actuator.j6_encoder.Update(globals->actuator.j6->pos_rad());
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }
  if ((loop_divisor % 100) == 1) { /** 5hz subloop **/
    globals->rpc.TickTimer();
    globals->rpc.KeepConnection();
    globals->rpc_connection_watcher.Update(globals->rpc.Connected());
  }
  if ((loop_divisor % 25) == 2) { /** 20hz subloop 1 **/
    host::ReportJointPosition(JointPosition{current_positions});
  }
}
}  // namespace

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();                 // 初始化全局对象
  host::Init(globals->rpc.rpc());  // 初始化RPC上位机通信
  fsm::Init();                     // 初始化状态机
  InitUserInput();                 // 初始化用户输入处理

  // 监听RPC连接状态变化，连接成功播放成功音效，断开连接播放错误音效
  globals->rpc_connection_watcher.OnValueChange([](const bool &old_value, const bool &new_value) {
    UNUSED(old_value);
    if (new_value) {
      globals->buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    } else {
      globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    }
  });

  // 监听设备状态变化，所有设备正常时LED为绿色呼吸灯，否则红灯闪烁
  globals->device_manager.OnDeviceStatusChange([](rm::device::Device *) {
    if (globals->device_manager.all_device_ok()) {
      globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    } else {
      globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    }
  });

  // 注册自定义控制器的接收回调，转发命令给状态机
  globals->custom_controller.OnRecv([&] {  //
    fsm::arm.receive(fsm::event::CustomCtrlCommand(globals->custom_controller.data));
  });

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

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
  // j6的回零位光电开关触发，通知状态机
  if (gpio_pin == GPIO_PIN_13) {
    fsm::arm.receive(fsm::event::OpticalSwitchTriggered{});
  }
}
}
