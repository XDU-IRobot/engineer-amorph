
#include "usbd_cdc_if.h"

#include "bsp/timer_task_scheduler.hpp"
#include "global_objects.hpp"
#include "shared_resources.hpp"

__attribute__((section(".sram4"))) SharedResourcesNoDtcm shared_no_dtcm;

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

void LeftArmLoop500hz() {}

void RightArmLoop300hz() { right_arm->Tick(); }

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

void LoggingLoop1hz() {
  etl::string<2048> buf;
  buf += "\r\n\r\n====== CHASSIS ======\r\n";
  buf += chassis->resources.device_manager.GetSummaryString();
  buf += "\r\n====== GIMBAL ======\r\n";
  buf += gimbal->resources.device_manager.GetSummaryString();
  buf += "\r\n====== RIGHT ARM ======\r\n";
  buf += right_arm->resources.device_manager.GetSummaryString();
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
  auto &shared = SharedResources::GetInstance();

  right_arm = new Arm{ArmConfig::MakeRight()};
  right_arm->Init();
  chassis = new Chassis{ChassisConfig::Make()};
  chassis->Init();
  gimbal = new Gimbal{GimbalConfig::Make()};
  gimbal->Init();

  // 创建调度器，绑定htim13
  static TimerTaskScheduler sched{&htim13};
  sched
      .AddTask(2000.f, etl::delegate<void()>::create<AhrsLoop1khz>(), "ahrs_1khz")
      // .AddTask(500.f, etl::delegate<void()>::create<LeftArmLoop500hz>(), "left_arm_500hz")
      .AddTask(300.f, etl::delegate<void()>::create<RightArmLoop300hz>(), "right_arm_300hz")
      .AddTask(500.f, etl::delegate<void()>::create<ChassisLoop500hz>(), "chassis_500hz")
      .AddTask(250.f, etl::delegate<void()>::create<GimbalLoop250hz>(), "gimbal_250hz")
      .AddTask(50.f, etl::delegate<void()>::create<SlowIoLoop50hz>(), "slow_io_50hz")
      .AddTask(125.f, etl::delegate<void()>::create<FastIoLoop125hz>(), "fast_io_125hz")
      .AddTask(1.f, etl::delegate<void()>::create<LoggingLoop1hz>(), "logging_1hz");
  sched.Start();

  shared.buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
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
