
#include <new>

#include "bsp/timer_task_scheduler.hpp"
#include "global_objects.hpp"
#include "shared_resources.hpp"
#include "cli/cli.hpp"
#include "orchestrator/orchestrator.hpp"
#include "client/packet_builder.hpp"
#include "ui/layout.hpp"
#include "bsp/reboot.hpp"

__attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

// 静态对象池，所有全局对象通过placement new在此分配
// 总大小在编译期确定，通过编译时的链接器输出可分析内存占用情况
alignas(std::max_align_t) static uint8_t
    g_static_pool[sizeof(SharedResources) + sizeof(Arm) * 2 + sizeof(Chassis) + sizeof(Gimbal) + sizeof(Orchestrator)];

Arm *left_arm{nullptr};
Arm *right_arm{nullptr};
Chassis *chassis{nullptr};
Gimbal *gimbal{nullptr};
Orchestrator *orch{nullptr};

namespace {
void AhrsLoop1khz() {
  auto &shared = SharedResources::GetInstance();
  shared.ahrs.Update();
}

void ArmLoop300hz() {
  auto &shared = SharedResources::GetInstance();
  shared.eu_storage.Update();
  left_arm->Tick();
  right_arm->Tick();
}

void ChassisLoop500hz() { chassis->Tick(); }

void GimbalLoop250hz() { gimbal->Tick(); }

volatile bool pending_mcp2515_restart = false;
void FastIoLoop125hz() {
  auto &shared = SharedResources::GetInstance();
  shared.rc.Update();
  shared.xbox_controller.Update();
  shared.vbus = shared_no_dtcm.vbus_adc.vbus();
  shared.device_manager.Update();
  shared.system_status = chassis->resources.device_manager.GetSummaryString();
  orch->Tick();

  // 按下图传遥控器的肩键时，重启MCU
  if (shared.vt_rc.online_status() == rm::device::Device::kOk && shared.vt_rc.data().trigger) {
    RebootMCU();
  }

  // 按下图传遥控器的暂停键时，重新初始化MCP2515
  if (shared.vt_rc.online_status() == rm::device::Device::kOk && shared.vt_rc.data().pause_button) {
    gimbal->fsm.receive(fsm_gimbal::event::ModeSwitch{fsm_gimbal::StateId::kNoForce});
    chassis->fsm.receive(fsm_chassis::event::ForceModeSwitch{fsm_chassis::StateId::kNoForce});
    pending_mcp2515_restart = true;
  }

  // // 无力时按左（Fn）键设置左臂j3零点
  // if (shared.vt_rc.online_status() == rm::device::Device::kOk &&  //
  //     shared.vt_rc.data().left_button &&                          //
  //     shared.vt_rc.data().switch_position == rm::device::VT03::SwitchPosition::N) {
  //   if (left_arm->fsm.get_state_id() == fsm_arm::StateId::kNoForce) {
  //     shared.buzzer_controller.Play<rm::modules::buzzer_melody::TheLick>();
  //     left_arm->resources.actuator.DmSetZero();
  //   }
  // }
  //
  // // 无力时按右（换相机）键设置右臂j3零点
  // if (shared.vt_rc.online_status() == rm::device::Device::kOk &&  //
  //     shared.vt_rc.data().right_button &&                         //
  //     shared.vt_rc.data().switch_position == rm::device::VT03::SwitchPosition::N) {
  //   if (right_arm->fsm.get_state_id() == fsm_arm::StateId::kNoForce) {
  //     shared.buzzer_controller.Play<rm::modules::buzzer_melody::TheLick>();
  //     right_arm->resources.actuator.DmSetZero();
  //   }
  // }
}

void SlowIoLoop50hz() {
  auto &shared = SharedResources::GetInstance();

  // 更新LED和蜂鸣器
  const auto &[r, g, b] = shared.led_controller.Update();
  shared.led.SetColor(r, g, b);
  shared.buzzer.SetFrequency(shared.buzzer_controller.Update().frequency);
}

void ClientLoop45hz() {
  auto &shared = SharedResources::GetInstance();

  bool left_gripper_open{};
  if (left_arm->resources.gripper_controller.state() == GripperController<>::kOpen) {
    left_gripper_open = true;
  } else if (left_arm->resources.gripper_controller.state() == GripperController<>::kClose) {
    left_gripper_open = false;
  }
  bool right_gripper_open{};
  if (right_arm->resources.gripper_controller.state() == GripperController<>::kOpen) {
    right_gripper_open = true;
  } else if (right_arm->resources.gripper_controller.state() == GripperController<>::kClose) {
    right_gripper_open = false;
  }

  amorph::schema::RobotFeedBack rf;
  const auto left_q = left_arm->resources.actuator.GetPositions();
  const auto right_q = right_arm->resources.actuator.GetPositions();
  for (int i = 0; i < 6; ++i) {
    rf.js.left[i] = left_q[i];
    rf.js.right[i] = right_q[i];
  }
  rf.js.left_gripper_open = left_gripper_open;
  rf.js.right_gripper_open = right_gripper_open;
  const auto is_dev_online = [](rm::device::Device &dev) -> bool {
    return dev.online_status() == rm::device::Device::kOk;
  };
  rf.ss.left_j1 = is_dev_online(left_arm->resources.actuator.j1);
  rf.ss.left_j2 = is_dev_online(left_arm->resources.actuator.j2);
  rf.ss.left_j3 = is_dev_online(left_arm->resources.actuator.j3);
  rf.ss.left_j4 = is_dev_online(left_arm->resources.actuator.j4);
  rf.ss.left_j5 = is_dev_online(left_arm->resources.actuator.j5);
  rf.ss.left_j6 = is_dev_online(left_arm->resources.actuator.j6);
  rf.ss.left_gripper = is_dev_online(left_arm->resources.actuator.gripper);
  rf.ss.right_j1 = is_dev_online(right_arm->resources.actuator.j1);
  rf.ss.right_j2 = is_dev_online(right_arm->resources.actuator.j2);
  rf.ss.right_j3 = is_dev_online(right_arm->resources.actuator.j3);
  rf.ss.right_j4 = is_dev_online(right_arm->resources.actuator.j4);
  rf.ss.right_j5 = is_dev_online(right_arm->resources.actuator.j5);
  rf.ss.right_j6 = is_dev_online(right_arm->resources.actuator.j6);
  rf.ss.right_gripper = is_dev_online(right_arm->resources.actuator.gripper);
  rf.ss.front_steer = is_dev_online(chassis->resources.f_steer);
  rf.ss.front_wheel = is_dev_online(chassis->resources.f_wheel);
  rf.ss.left_steer = is_dev_online(chassis->resources.l_steer);
  rf.ss.left_wheel = is_dev_online(chassis->resources.l_wheel);
  rf.ss.right_steer = is_dev_online(chassis->resources.r_steer);
  rf.ss.right_wheel = is_dev_online(chassis->resources.r_wheel);
  // rf.ss.gimbal_yaw = is_dev_online(gimbal->resources.yaw_motor);
  rf.ss.camera_lift = is_dev_online(gimbal->resources.lift_motor);
  rf.ss.left_j3_temp = left_arm->resources.actuator.j3.coil_temperature();
  rf.ss.right_j3_temp = right_arm->resources.actuator.j3.coil_temperature();
  rf.ss.left_gripper_temp = left_arm->resources.actuator.gripper.coil_temperature();
  rf.ss.right_gripper_temp = right_arm->resources.actuator.gripper.coil_temperature();
  rf.ss.vehicle_config = orch->fsm.get_state_id();
  rf.ss.left_arm_fsm_state = left_arm->fsm.get_state_id();
  rf.ss.right_arm_fsm_state = right_arm->fsm.get_state_id();
  std::memset(&rf.eud, 0, sizeof(rf.eud));
  if (is_dev_online(shared.vision_jetson)) {
    for (int i = 0; i < 8; i++) {
      rf.eud[i] = shared.vision_jetson.data()[i];
    }
  } else {
    std::memset(&rf.eud, 0, sizeof(rf.eud));
  }
  static PacketBuilder::Packet client_packet = PacketBuilder::Build(rf);
  client_packet = PacketBuilder::Build(rf);
  shared.no_dtcm->vt_link_serial.WriteAsync((uint8_t *)&client_packet, sizeof(client_packet), [] {});
}

void UiLoop28hz() {
  auto &shared = SharedResources::GetInstance();

  if (shared.ref_pm.online_status() == rm::device::Device::kOk) {
    ui::SetRobotId(shared.ref_pm.data().robot_status.robot_id);
  }
  shared.ui_ts.schedule();
}

void LoggingLoop1hz() {
  auto &shared = SharedResources::GetInstance();

  etl::string<256> buf;
  const auto q = right_arm->resources.actuator.GetPositions();
  const auto qd = right_arm->resources.actuator.GetVelocities();
  auto eff = right_arm->resources.actuator.GetEfforts();

  // 前三个关节的 CSV 行: tick,q0,q1,q2,qd0,qd1,qd2,eff0,eff1,eff2
  char tmp[40];
  sprintf(tmp, "%f", HAL_GetTick() / 1000.f);
  buf += tmp;
  for (int i = 0; i < 3; ++i) {
    buf += ',';
    sprintf(tmp, "%.4f", q[i]);
    buf += tmp;
  }
  for (int i = 0; i < 3; ++i) {
    buf += ',';
    sprintf(tmp, "%.4f", qd[i]);
    buf += tmp;
  }
  for (int i = 0; i < 3; ++i) {
    buf += ',';
    sprintf(tmp, "%.4f", eff[i]);
    buf += tmp;
  }
  buf += "\r\n";

  // shared.no_dtcm->cli_serial.Write((uint8_t *)buf.c_str(), buf.length(), 1);
}
}  // namespace

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MCP_INT_Pin) {
    SharedResources::GetInstance().mcp_can.ExtiIrqHandler();
  }
}
[[noreturn]] void AppMain(void) {
  uint8_t *pool = g_static_pool;

  SharedResources::PlaceAt(pool);
  pool += sizeof(SharedResources);
  auto &shared = SharedResources::GetInstance();

  left_arm = new (pool) Arm{ArmConfig::MakeLeft()};
  pool += sizeof(Arm);
  left_arm->Init();
  right_arm = new (pool) Arm{ArmConfig::MakeRight()};
  pool += sizeof(Arm);
  right_arm->Init();
  chassis = new (pool) Chassis{ChassisConfig::Make()};
  pool += sizeof(Chassis);
  chassis->Init();
  gimbal = new (pool) Gimbal{GimbalConfig::Make()};
  pool += sizeof(Gimbal);
  gimbal->Init();
  orch = new (pool) Orchestrator;
  orch->Init();

  ui::Init();

  // 创建调度器，绑定htim13
  static TimerTaskScheduler sched{&htim13};
  sched.AddTask(1000.f, etl::delegate<void()>::create<AhrsLoop1khz>(), "ahrs_1khz")
      .AddTask(300.f, etl::delegate<void()>::create<ArmLoop300hz>(), "arm_300hz")
      .AddTask(500.f, etl::delegate<void()>::create<ChassisLoop500hz>(), "chassis_500hz")
      .AddTask(250.f, etl::delegate<void()>::create<GimbalLoop250hz>(), "gimbal_250hz")
      .AddTask(50.f, etl::delegate<void()>::create<SlowIoLoop50hz>(), "slow_io_50hz")
      .AddTask(45.f, etl::delegate<void()>::create<ClientLoop45hz>(), "client_45hz")
      .AddTask(125.f, etl::delegate<void()>::create<FastIoLoop125hz>(), "fast_io_125hz")
      .AddTask(28.f, etl::delegate<void()>::create<UiLoop28hz>(), "ui_28hz");
  // .AddTask(20.f, etl::delegate<void()>::create<LoggingLoop1hz>(), "logging_1hz");
  sched.Start();

  shared.buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  cli::Init();

  for (;;) {
    cli::Process();
    if (pending_mcp2515_restart) {
      pending_mcp2515_restart = false;
      shared.mcp_can.Stop();
      shared.mcp_can.Begin();
    }
    // 处理CAN定频发送
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
