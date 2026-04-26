
#include "bsp/timer_task.hpp"
#include "global_objects.hpp"
#include "shared_resources.hpp"

__attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

namespace {
void SubLoop500hz() {
  auto &shared = SharedResources::GetInstance();

  shared.rc.Update();
  shared.vbus = shared_no_dtcm.vbus_adc.vbus();

  chassis->Tick();

  shared.system_status = chassis->resources.device_manager.GetSummaryString();
}

void SubLoop50hz() {
  auto &shared = SharedResources::GetInstance();

  shared.servo.ReadPosition();

  // 更新LED和蜂鸣器
  const auto &[r, g, b] = shared.led_controller.Update();
  shared.led.SetColor(r, g, b);
  shared.buzzer.SetFrequency(shared.buzzer_controller.Update().frequency);
}

void AhrsLoop2khz() {
  auto &shared = SharedResources::GetInstance();
  shared.imu.Update();
}
}  // namespace

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MCP_INT_Pin) {
    SharedResources::GetInstance().mcp_can.ExtiIrqHandler();
  }
}

[[noreturn]] void AppMain(void) {
  auto &shared = SharedResources::GetInstance();

  chassis = new Chassis{ChassisConfig::Make()};
  chassis->Init();

  // // 定义键盘键位
  // S.kh.OnKeyDown(rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kF,
  //                [](const rm::modules::KeyEvent &e) {
  //                  // 杆升
  //                  fsm_gimbal::gimbal.receive(fsm_gimbal::event::KeyboardCommand{true});
  //                })
  //     .OnKeyDown(rm::device::RefereeProtocol<rm::device::RefereeRevision::kNewV110>::RemoteControl::kC,
  //                [](const rm::modules::KeyEvent &e) {
  //                  // 杆降
  //                  fsm_gimbal::gimbal.receive(fsm_gimbal::event::KeyboardCommand{false});
  //                });

  // // 监听设备状态变化，所有设备正常时LED为绿色呼吸灯，否则红灯闪烁
  // S.device_manager.OnDeviceStatusChange([&](rm::device::Device *) {
  //   if (S.device_manager.all_device_ok()) {
  //     S.led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
  //   } else {
  //     S.led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  //   }
  // });

  shared.ref_vt_link.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
    UNUSED(seq);
    switch (cmd_id) {
      case rm::device::RefereeCmdId<rm::device::RefereeRevision::kNewV110>::kCustomRobotData: {
        // 从图传链路解包出自定义控制器数据，转发到CAN总线上
        shared.custom_ctrl_device.ReportStatus(rm::device::Device::kOk);
        break;
      }
      case rm::device::RefereeCmdId<rm::device::RefereeRevision::kNewV110>::kRemoteControl: {
        shared.vt_remote_device.ReportStatus(rm::device::Device::kOk);
        break;
      }
      default: {
        break;
      }
    }
  });  // 注册解包回调
  // 初始化图传链路串口
  rm::hal::SerialRxCallbackFunction on_vt_serial_rx = [&](etl::span<const uint8_t> data) {
    for (auto byte : data) {
      shared.ref_vt_link << byte;
    }
  };
  shared_no_dtcm.vt_link_serial_7.AttachRxCallback(on_vt_serial_rx);  // 注册接收回调，把收到的数据传给解码器
  shared_no_dtcm.vt_link_serial_7.Start();

  // 创建调度器，绑定htim13
  static TimerTaskScheduler sched{&htim13};
  sched.AddTask(2000.f, etl::delegate<void()>::create<AhrsLoop2khz>(), "ahrs_2khz")
      .AddTask(500.f, etl::delegate<void()>::create<SubLoop500hz>(), "state_500hz")
      .AddTask(50.f, etl::delegate<void()>::create<SubLoop50hz>(), "io_50hz");
  sched.Start();

  shared.led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  shared.buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
    shared.left_can.Process();
    shared.right_can.Process();
    shared.comm_can.Process();
    shared.mcp_can.Process();
    // 禁用 EXTI 中断，统一执行以下两项维护操作，防止与 Write() 中的 PRIMASK
    // 临界区以外的残余窗口产生 SPI 竞争：
    //   1. Bus-Off 恢复：检测 EFLG.TXBO/TXEP 并在必要时重新初始化 MCP2515
    //   2. 兜底轮询：若 MCU 错过了下降沿，INT 引脚仍会持续拉低，此处补调一次
    HAL_NVIC_DisableIRQ(MCP_INT_EXTI_IRQn);
    shared.mcp_can.CheckAndRecover();
    if (HAL_GPIO_ReadPin(MCP_INT_GPIO_Port, MCP_INT_Pin) == GPIO_PIN_RESET) {
      shared.mcp_can.ExtiIrqHandler();
    }
    HAL_NVIC_EnableIRQ(MCP_INT_EXTI_IRQn);
  }
}
}
