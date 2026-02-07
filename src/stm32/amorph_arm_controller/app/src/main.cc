
#include "fsm.hpp"
#include "rpc/host_api.hpp"
#include "bsp/timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

std::array<float, 7> post_positions{};

namespace {
void MainLoop() {
  globals->device_manager.Update();
  globals->rc.Update();
  globals->actuator.UpdateEncoderCounter();
  fsm::arm.receive(fsm::event::ControlLoop{globals->actuator.GetPositions(), globals->actuator.GetVelocities()});
  post_positions = globals->actuator.GetPositions();

  static int loop_divisor;
  loop_divisor = (loop_divisor + 1) % 100;

  if ((loop_divisor % 10) == 0) { /** 50hz subloop **/
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led.SetColor(r, g, b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }
  if ((loop_divisor % 25) == 1) { /** 20hz subloop 1 **/
    globals->rpc.Poll();
    globals->rpc_connection_watcher.Update(globals->rpc.Connected());
  }
  if ((loop_divisor % 25) == 2) { /** 20hz subloop 2 **/
    host::ReportJointPosition(JointPosition{globals->actuator.GetPositions()});
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

  // // 设零点
  // HAL_Delay(1500);  // 达妙电机开机需要时间
  // globals->actuator.DmSetZero();

  // 监听RPC连接状态变化，连接成功播放成功音效，断开连接播放错误音效
  globals->rpc_connection_watcher.OnValueChange([](const bool &old_value, const bool &new_value) {
    UNUSED(old_value);
    if (new_value) {
      globals->buzzer_controller.Play<rm::modules::buzzer_melody::Success>();
    } else {
      globals->buzzer_controller.Play<rm::modules::buzzer_melody::Error>();
    }
  });

  // 订阅上位机下发的轨迹指令
  globals->rpc.rpc()->subscribe("execution_request", [&](const JointTrajectory &traj) {
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    fsm::arm.receive(fsm::event::ExecutionReq{traj});
  });

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
