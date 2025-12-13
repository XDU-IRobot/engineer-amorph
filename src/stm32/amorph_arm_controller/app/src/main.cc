
#include <etl/cyclic_value.h>

#include "fsm.hpp"
#include "rpc/host_api.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

namespace {
void MainLoop() {
  static etl::cyclic_value<int, 0, 500> loop_divisor;
  ++loop_divisor;
  if ((loop_divisor % 10) == 0) {
    // 50hz subloop
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }
  // if ((loop_divisor % 500) == 0) {
  //   // 2hz subloop
  //   host::L::I("hello!!");
  //   // globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
  // }

  globals->rc.Update();

  const std::array<float, 6> current_positions =  //
      {globals->actuator.j1->pos(),
       globals->actuator.j2->pos(),
       globals->actuator.j3->pos() - globals->actuator.j2->pos(),  //
       globals->actuator.j4->pos(),
       globals->actuator.j5->pos(),
       globals->actuator.j6->pos_rad()};
  const std::array<float, 6> current_velocities =                  //
      {globals->actuator.j1->vel(),                                //
       globals->actuator.j2->vel(),                                //
       globals->actuator.j3->vel() - globals->actuator.j2->vel(),  //
       globals->actuator.j4->vel(),                                //
       globals->actuator.j5->vel(),                                //
       globals->actuator.j6->rpm() / 60.f * 2.f * std::numbers::pi_v<float>};
  fsm::arm.receive(fsm::event::ControlLoop{current_positions, current_velocities});
}
}  // namespace

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();                 // 初始化全局对象
  host::Init(globals->rpc.rpc());  // 初始化RPC上位机通信
  fsm::Init();                     // 初始化状态机
  InitUserInput();                 // 初始化用户输入处理

  // 创建主循环定时任务，定频500hz
  TimerTask mainloop{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop.Start();

  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  for (;;) {
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
  // j6的回零位光电开关触发，通知状态机
  if (gpio_pin == GPIO_PIN_13) {
    globals->buzzer_controller.Play<buzzer_melody::HiBeeps<1>>();
    fsm::arm.receive(fsm::event::OpticalSwitchTriggered{});
  }
}
}