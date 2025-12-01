
#include <numbers>

#include <etl/cyclic_value.h>

#include "tim.h"

#include "rpc/host_api.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"
#include "fsm.hpp"

Globals *globals{nullptr};

static void MainLoop() {
  static etl::cyclic_value<int, 0, 1000> loop_divisor;
  ++loop_divisor;
  if ((loop_divisor % 20) == 0) {
    // 50hz subloop
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }
  if ((loop_divisor % 500) == 0) {
    // 2hz subloop
    host::L::I("hello!!");
    // globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
  }

  globals->rc_sd_watcher.Update(globals->rc.channel(11));

  const auto current_positions =  //
      std::array<float, 6>{globals->actuator.j1->pos(),
                           globals->actuator.j2->pos(),
                           globals->actuator.j3->pos() - globals->actuator.j2->pos(),  //
                           globals->actuator.j4->pos(),
                           globals->actuator.j5->pos(),
                           globals->actuator.j6->pos_rad()};
  const auto current_velocities =                                                      //
      std::array<float, 6>{globals->actuator.j1->vel(),                                //
                           globals->actuator.j2->vel(),                                //
                           globals->actuator.j3->vel() - globals->actuator.j2->vel(),  //
                           globals->actuator.j4->vel(),                                //
                           globals->actuator.j5->vel(),                                //
                           globals->actuator.j6->rpm() / 60.f * 2.f * std::numbers::pi_v<float>};
  fsm::arm.receive(fsm::event::ControlLoop{current_positions, current_velocities});
}

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();                 // 初始化全局对象
  host::Init(globals->rpc.rpc());  // 初始化RPC上位机通信
  fsm::Init();                     // 初始化状态机

  // 监听遥控器拨杆
  globals->rc_sd_watcher.OnValueChange([](const int &old_value, const int &new_value) {
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 500) {
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    }
    if (new_value <= 500) {
      fsm::arm.receive(fsm::event::FaultClear{});
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kIdle});
    }
  });

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.Start();

  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  for (;;) {
  }
}
}