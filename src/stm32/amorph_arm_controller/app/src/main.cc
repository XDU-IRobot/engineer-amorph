
#include "fsm.hpp"
#include "rpc/host_api.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

etl::fsm_state_id_t current_state;

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->host_connection_watcher.Update();
  globals->rc.Update();
  globals->actuator.UpdateEncoderCounter();

  // 在主循环上下文中处理CAN接收到的上位机命令，避免在中断中直接调用状态机
  globals->host.ProcessPendingCommand();

  current_state = fsm::arm.get_state_id();
  fsm::arm.receive(fsm::event::ControlLoop{globals->actuator.GetPositions(), globals->actuator.GetVelocities()});

  static int loop_divisor;
  loop_divisor = (loop_divisor + 1) % 100;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    // 更新LED灯和蜂鸣器状态
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);

    // 向上位机发送反馈
    globals->host.tx_data_.mode = static_cast<Stm32Feedback::Mode>(fsm::arm.get_state_id());
    const auto current_pos = globals->actuator.GetPositions();
    const auto current_vel = globals->actuator.GetVelocities();
    std::memcpy(globals->host.tx_data_.joint_positions, current_pos.data(), sizeof(float) * 6);
    std::memcpy(globals->host.tx_data_.joint_velocities, current_vel.data(), sizeof(float) * 6);
    globals->host.tx_data_.gripper_open = false;
    globals->host.Write();
  }
  if ((loop_divisor % 25) == 1) { /** 20hz subloop 1 **/
  }
  // if ((loop_divisor % 25) == 2) { /** 20hz subloop 2 **/
  //   host::ReportJointPosition(JointPosition{globals->actuator.GetPositions()});
  // }
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
  rm::hal::SerialRxCallbackFunction on_vt_serial_rx = [&](const std::vector<uint8_t> &data, uint16_t len) {
    for (int i = 0; i < len; ++i) {
      globals->ref << data[i];
    }
  };
  globals->vt_serial.AttachRxCallback(on_vt_serial_rx);  // 注册接收回调，把收到的数据传给解码器
  globals->ref.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
    UNUSED(seq);
    switch (cmd_id) {
      case rm::device::RefereeCmdId<rm::device::RefereeRevision::kV170>::kCustomRobotData: {
        // 把收到的数据转发给状态机处理
        CustomControllerCommand cmd;
        std::memcpy(&cmd, globals->ref.data().custom_robot_data.data, sizeof(cmd));
        fsm::arm.receive(fsm::event::CustomCtrlCommand{cmd});
        break;
      }
      default: {
        break;
      }
    }
  });                          // 注册解包回调，收到自定义控制器数据时转发命令给状态机
  globals->vt_serial.Begin();  // 启动串口接收

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
