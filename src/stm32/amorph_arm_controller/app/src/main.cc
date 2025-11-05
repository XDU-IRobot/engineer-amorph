
#include <etl/cyclic_value.h>

#include "tim.h"

#include "timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

static void MainLoop() {
  static etl::cyclic_value<int, 0, 100> prescale_10hz;
  static etl::cyclic_value<int, 0, 1000> prescale_1hz;

  JointPosition jp;
  jp.positions = {1.0f, 2.0f, 3.0f};

  // 10hz维护RPC连接
  ++prescale_10hz;
  ++prescale_1hz;
  if (prescale_10hz.get() == 0) {
    globals->rpc.KeepConnection();
    globals->rpc.TickTimer();
  }
  if (prescale_1hz.get() == 0) {
    globals->rpc.ReportJointPosition(jp);
    globals->rpc.Info("asiurhfgiaushgaighaispghasigh!!!!!!!hapgidsrhgoiahopgahiorghadsrhiogaio##");
  }
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
  for (;;) {
    __WFI();
  }
}
}