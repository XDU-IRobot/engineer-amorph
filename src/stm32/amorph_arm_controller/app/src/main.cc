
#include "tim.h"

#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};
ArmController test_ct;

static void MainLoop() {
  static int subloop_divisor = 0;
  subloop_divisor = (subloop_divisor + 1) % 1000;
  if ((subloop_divisor % 20) == 0) {
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }
  globals->rc_sd_watcher.Update(globals->rc.channel(11));
  test_ct.Update({0.f, globals->actuator.j1->pos(), globals->actuator.j2->pos(), 0.f, 0.f, 0.f},  //
                 {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},                                                  //
                 0.001f);

  if (test_ct.status() == ArmController::kFault || test_ct.status() == ArmController::kDisabled) {
    globals->actuator.j1->SetPosition(0, 0, 0, 0, 0);
  } else {
    globals->actuator.j1->SetPosition(test_ct.output().positions[1], 0, 0, 7.5f, 2.0f);
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

  globals->rc_sd_watcher.OnValueChange(
      etl::delegate<void(const int &, const int &)>::create([](const int &old_value, const int &new_value) {
        if (old_value == 0) {  // 第一次触发不响应
          return;
        }
        globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
        if (new_value > 500) {
          globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
          test_ct.Enable(false);

          globals->actuator.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
          globals->actuator.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        }
        if (new_value <= 500) {
          globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
          test_ct.FaultClear();
          test_ct.Enable(true);

          globals->actuator.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          globals->actuator.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);

          JointTrajectoryPoint p;
          p.positions = {2.f, 1.75f, 2.f, 2.f, 2.f, 2.f};
          p.velocities.resize(6);
          test_ct.GoTo(p);
        }
      }));

  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  for (;;) {
    __WFI();
  }
}
}