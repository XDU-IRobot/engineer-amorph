
#include "tim.h"

#include "timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

static void MainLoop() {
  const auto &[r, g, b] = globals->led_controller.Update();
  globals->led.SetColor(r, g, b);
}

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.Start();

  globals->buzzer.Beep(2);
  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  for (;;) {
    __WFI();
  }
}
}