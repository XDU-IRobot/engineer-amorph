/**
 * @file  main.cc
 * @brief 单板双臂控制器主入口
 *
 * 左臂电机 CAN：FDCAN1（lower）+ FDCAN2（upper）
 * 右臂电机 CAN：FDCAN2（lower）+ FDCAN1（upper）
 * 通信  CAN：FDCAN3（与上位机 / 底盘）
 *
 * 注意：两臂共用同两根 CAN 总线，电机 CAN ID 必须互不冲突。
 *       在 arm_config.cc 中为右臂分配不同 ID 段。
 */

#include "arm_instance.hpp"
#include "arm_config.hpp"
#include "bsp/timer_task.hpp"

static SharedResources* S{nullptr};
static ArmInstance*     arms[2]{};  // arms[0]=左臂  arms[1]=右臂

// ============================================================
//  500 Hz 主循环
// ============================================================
static void MainLoop() {
  static volatile uint32_t loop_us = 0;
  const uint32_t t0      = __HAL_TIM_GET_COUNTER(&htim12);
  const uint32_t tim_max = __HAL_TIM_GET_AUTORELOAD(&htim12) + 1;

  S->rc.UpdateSwitchWatchers();
  S->imu.Update();

  for (auto* a : arms) a->Tick();

  static int div = 0;
  if (++div >= 10) {  // 50 Hz 子循环
    div = 0;
    const auto& [r, g, b] = S->led_controller.Update();
    S->led.SetColor(r, g, b);
    S->buzzer.SetFrequency(S->buzzer_controller.Update().frequency);
    for (auto* a : arms) a->TickSlow();
  }

  const uint32_t t1 = __HAL_TIM_GET_COUNTER(&htim12);
  loop_us = (t1 >= t0) ? (t1 - t0) : (tim_max + t1 - t0);
}

static void BindRc(ArmInstance& a) {
  S->rc(a.rc_ch_mode_switch).OnValueChange([&a](const int& old_v, const int& new_v) {
    if (old_v == 0) return;
    if (new_v > 1300 && new_v < 1700) {
      a.arm_fsm.receive(fsm::event::FaultClear{});
      a.arm_fsm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    } else if (new_v > 0 && new_v < 500) {
      a.arm_fsm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kIdle});
    } else if (new_v > 500 && new_v < 1100) {
      a.arm_fsm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kLongJointCalibrating});
    }
  });
  S->rc(a.rc_ch_next_joint).OnValueChange([&a](const int&, const int& new_v) {
    if (new_v > 1000) a.arm_fsm.receive(fsm::event::NextJoint{});
  });
}

// ============================================================
//  应用入口
// ============================================================
extern "C" [[noreturn]] void AppMain(void) {
  S = new SharedResources;
  S->Init();
  S->rc.SetName("rc");

  // 创建两个臂实例（共享 FDCAN1/FDCAN2，仅初始化一次）
  arms[0] = new ArmInstance{MakeLeftArmConfig (S->fdcan1_bus, S->fdcan2_bus), *S};
  arms[1] = new ArmInstance{MakeRightArmConfig(S->fdcan2_bus, S->fdcan1_bus), *S};
  S->fdcan1_bus.SetFilter(0, 0); S->fdcan1_bus.Begin();
  S->fdcan2_bus.SetFilter(0, 0); S->fdcan2_bus.Begin();

  for (auto* a : arms) {
    a->Init(*S);
    BindRc(*a);
    a->host.OnRx([a](const HostCommand& cmd) {
      a->arm_fsm.receive(fsm::event::HostCommand{cmd});
    });
    a->device_manager.OnDeviceStatusChange([](rm::device::Device*) {
      const bool ok = arms[0]->device_manager.all_device_ok() &&
                      arms[1]->device_manager.all_device_ok();
      if (ok) S->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
      else     S->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    });
  }

  // 图传链路自定义控制（广播给两臂）
  S->vt_link_receiver.OnCustomCtrlRecv([]() {
    static CustomControllerCommand cmd;
    std::memcpy(&cmd, &S->vt_link_receiver.custom_ctrl_data(), sizeof(cmd));
    for (auto* a : arms) a->arm_fsm.receive(fsm::event::CustomCtrlCommand{cmd});
  });
  S->vt_link_receiver.OnCartesianCustomCtrlRecv([]() {
    static CartesianCustomControllerCommand cmd;
    std::memcpy(&cmd, &S->vt_link_receiver.cartesian_custom_ctrl_data(), sizeof(cmd));
    for (auto* a : arms) a->arm_fsm.receive(fsm::event::CartesianCustomCtrlCommand{cmd});
  });

  TimerTask mainloop{&htim13, etl::delegate<void()>::create<MainLoop>()};
  mainloop.Start();
  S->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
    S->fdcan1_bus.Process();
    S->fdcan2_bus.Process();
    S->comm_can.Process();
  }
}
