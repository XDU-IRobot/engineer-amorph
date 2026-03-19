
#include "fsm.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->host_connection_watcher.Update();
  globals->rc.UpdateSwitchWatchers();
  globals->actuator.UpdateEncoderCounter();

  fsm::arm.receive(fsm::event::ControlLoop{globals->actuator.GetPositions(), globals->actuator.GetVelocities()});

  static volatile int loop_divisor;
  loop_divisor = (loop_divisor + 1) % 100;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    // 更新LED灯和蜂鸣器状态
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);

    // 向上位机发送反馈
    globals->host.tx_data_.mode = static_cast<Stm32FeedbackCompressed::Mode>(fsm::arm.get_state_id());
    const auto current_pos = globals->actuator.GetPositions();
    const auto current_vel = globals->actuator.GetVelocities();
    const auto current_effort = globals->actuator.GetEfforts();
    std::memcpy(globals->host.tx_data_.joint_positions, current_pos.data(), sizeof(float) * 6);
    std::memcpy(globals->host.tx_data_.joint_velocities, current_vel.data(), sizeof(float) * 6);
    std::memcpy(globals->host.tx_data_.joint_efforts, current_effort.data(), sizeof(float) * 6);
    globals->host.tx_data_.gripper_state =
        static_cast<Stm32FeedbackCompressed::GripperState>(globals->arm_controller.gripper().state());
    globals->host.Write();
  }
  if ((loop_divisor % 25) == 1) { /** 20hz subloop 1 **/
  }
}

void InitUserInput() {
  // 1. 定义遥控器拨杆功能
#if defined(I_AM_LEFT_ARM)
  constexpr int kModeSwitch = RcReceiver::kSb;
#elif defined(I_AM_RIGHT_ARM)
  constexpr int kModeSwitch = RcReceiver::kSc;
#endif
  globals->rc(kModeSwitch).OnValueChange([](const int &old_value, const int &new_value) {
    // 353-up, 1024-mid, 1695-down
    if (old_value == 0) {
      return;  // 第一次触发不响应
    }
    if (new_value > 1300 && new_value < 1700) {  // 机械臂模式杆在下 - 无力
      fsm::arm.receive(fsm::event::FaultClear{});
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
    }
    if (new_value > 0 && new_value < 500) {  // 机械臂模式杆在上 - 自动模式，idle
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kIdle});
    }
    if (new_value > 500 && new_value < 1100) {  // 机械臂模式杆在中 - 手动调试模式
      fsm::arm.receive(fsm::event::ForceModeSwitch{fsm::StateId::kLongJointCalibrating});
    }
  });
  globals->rc(RcReceiver::kSh).OnValueChange([](const int &old_value, const int &new_value) {
    UNUSED(old_value);
    if (new_value > 1000) {  // SH按下
      fsm::arm.receive(fsm::event::NextJoint{});
    }
  });
  globals->rc(RcReceiver::kSg).OnValueChange([](const int &old_value, const int &new_value) { UNUSED(old_value); });
}
}  // namespace

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();  // 初始化全局对象
  fsm::Init();      // 初始化状态机
  InitUserInput();  // 初始化用户输入处理

  // // 设零点
  // HAL_Delay(1500);  // 达妙电机开机需要时间
  // globals->actuator.DmSetZero();

  // 订阅上位机通过CANFD发送的控制命令，转发给状态机处理
  globals->host.OnRx([](const HostCommand &cmd) { fsm::arm.receive(fsm::event::HostCommand{cmd}); });

  // 监听设备状态变化，所有设备正常时LED为绿色呼吸灯，否则红灯闪烁
  globals->device_manager.OnDeviceStatusChange([](rm::device::Device *) {
    if (globals->device_manager.all_device_ok()) {
      globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    } else {
      globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    }
  });

  // 初始化图传链路串口
  globals->vt_link_receiver.OnCustomCtrlRecv([&]() {
    // 把收到的数据转发给状态机处理
    static CustomControllerCommand cc_cmd;
    const auto &data = globals->vt_link_receiver.custom_ctrl_data();
    std::memcpy(&cc_cmd, &data, sizeof(cc_cmd));
    fsm::arm.receive(fsm::event::CustomCtrlCommand{cc_cmd});
  });  // 注册解包回调，收到自定义控制器数据时转发命令给状态机

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
