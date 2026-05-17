
#include "usbd_cdc_if.h"

#include <new>

#include "bsp/timer_task_scheduler.hpp"
#include "global_objects.hpp"
#include "shared_resources.hpp"
#include "subsystem_compose.hpp"
#include "ui/serial_tx.hpp"
#include "ui/ui.h"

__attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

// 静态对象池，所有全局对象通过placement new在此分配
// 总大小在编译期确定，通过编译时的链接器输出可分析内存占用情况
alignas(std::max_align_t) static uint8_t
    g_static_pool[sizeof(SharedResources) + sizeof(Arm) * 2 + sizeof(Chassis) + sizeof(Gimbal)];

Arm *left_arm{nullptr};
Arm *right_arm{nullptr};
Chassis *chassis{nullptr};
Gimbal *gimbal{nullptr};

namespace {
void AhrsLoop1khz() {
  auto &shared = SharedResources::GetInstance();
  shared.onboard_imu.Update();
  shared.mahony_ahrs.Update(rm::modules::ImuData6Dof{
      .gx = shared.onboard_imu.gyro_x(),
      .gy = shared.onboard_imu.gyro_y(),
      .gz = shared.onboard_imu.gyro_z(),
      .ax = shared.onboard_imu.accel_x(),
      .ay = shared.onboard_imu.accel_y(),
      .az = shared.onboard_imu.accel_z(),
  });
}

void ArmLoop300hz() {
  left_arm->Tick();
  right_arm->Tick();
}

void ChassisLoop500hz() { chassis->Tick(); }

void GimbalLoop250hz() { gimbal->Tick(); }

void FastIoLoop125hz() {
  auto &shared = SharedResources::GetInstance();
  shared.rc.Update();
  shared.vbus = shared_no_dtcm.vbus_adc.vbus();
  shared.device_manager.Update();
  shared.system_status = chassis->resources.device_manager.GetSummaryString();

  // 按下图传遥控器的暂停键时，重启MCU
  if (shared.vt_rc.data().pause_button) {
    __ASM volatile("cpsid i");  // 进入临界区，禁止所有中断
    NVIC_SystemReset();         // 重启
  }
}

void SlowIoLoop50hz() {
  auto &shared = SharedResources::GetInstance();
  // 更新LED和蜂鸣器
  const auto &[r, g, b] = shared.led_controller.Update();
  shared.led.SetColor(r, g, b);
  shared.buzzer.SetFrequency(shared.buzzer_controller.Update().frequency);
}

void UiLoop28hz() {
  auto &shared = SharedResources::GetInstance();
  static int round_robin = 0;

  if (HAL_UART_GetState(&huart10) & 0x01U) {
    return;  // 若串口忙则跳过本轮
  }

  // update id
  ui_self_id = shared.ref_pm.data().robot_status.robot_id;

  // chassis orientation & is enable
  const float chassis_ori =
      gimbal->resources.yaw_calib.RealToLogical(gimbal->resources.yaw_motor.pos_rad()) / M_PIf * 180.f;
  ui_g_g_main_chassis_ori->start_angle = rm::modules::Wrap(chassis_ori - 30.f, 0.f, 360.f);
  ui_g_g_main_chassis_ori->end_angle = rm::modules::Wrap(chassis_ori + 30.f, 0.f, 360.f);
  if (chassis->fsm.get_state_id() == fsm_chassis::StateId::kNoForce) {
    ui_g_g_main_chassis_ori->color = 7;
  } else {
    ui_g_g_main_chassis_ori->color = 2;
  }

  // can pass?
  const bool can_pass = gimbal->fsm.get_state_id() == fsm_gimbal::StateId::kDown;
  if (can_pass) {
    ui_g_g_main_cross_1->width = 0;
    ui_g_g_main_cross_2->width = 0;
  } else {
    ui_g_g_main_cross_1->width = 5;
    ui_g_g_main_cross_2->width = 5;
  }

  ui_update_g();
  UiProcessTx();
}

void LoggingLoop1hz() {
  etl::string<2048> buf;
  const auto q = left_arm->resources.actuator.GetPositions();
  const auto qd = left_arm->resources.actuator.GetVelocities();
  auto eff = left_arm->resources.actuator.GetEfforts();
  eff[2] *= 2.f;

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

  CDC_Transmit_HS((uint8_t *)buf.c_str(), buf.length());
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

  UiInit();

  // 创建调度器，绑定htim13
  static TimerTaskScheduler sched{&htim13};
  sched.AddTask(1000.f, etl::delegate<void()>::create<AhrsLoop1khz>(), "ahrs_1khz")
      .AddTask(300.f, etl::delegate<void()>::create<ArmLoop300hz>(), "arm_300hz")
      .AddTask(500.f, etl::delegate<void()>::create<ChassisLoop500hz>(), "chassis_500hz")
      .AddTask(250.f, etl::delegate<void()>::create<GimbalLoop250hz>(), "gimbal_250hz")
      .AddTask(50.f, etl::delegate<void()>::create<SlowIoLoop50hz>(), "slow_io_50hz")
      .AddTask(125.f, etl::delegate<void()>::create<FastIoLoop125hz>(), "fast_io_125hz")
      .AddTask(28.f, etl::delegate<void()>::create<UiLoop28hz>(), "ui_28hz")
      .AddTask(20.f, etl::delegate<void()>::create<LoggingLoop1hz>(), "logging_1hz");
  sched.Start();

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
