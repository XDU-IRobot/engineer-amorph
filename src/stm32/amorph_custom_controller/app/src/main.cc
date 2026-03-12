
#include <librm.hpp>

#include <etl/pseudo_moving_average.h>

#include <lvgl.h>

#include "adc.h"
#include "arm_ui.hpp"
#include "buzzer.hpp"
#include "globals.hpp"
#include "packet_builder.hpp"
#include "timer_task.hpp"
#include "st7789_driver.h"

namespace {

bool g_comp_enable = false;

volatile uint32_t g_adc1_val = 0;

/* ── MainLoop 执行时间统计 ───────────────────────────────────────────── */
// 使用 DWT->CYCCNT 实现 CPU 周期级计时，再换算成微秒
// etl::pseudo_moving_average<T, WINDOW> 对采样值做滑动平均平滑
static etl::pseudo_moving_average<uint32_t, 16> g_mainloop_time_avg{0u};

inline void DwtInit() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

inline uint32_t DwtCycleToUs(uint32_t cycles) {
  // STM32F427 主频 168 MHz
  return cycles / 168u;
}

void MainLoop() {
  const uint32_t t_start = DWT->CYCCNT;
  globals->motors.j5_encoder.Update((2.0f * M_PI) - (static_cast<float>(g_adc1_val) / 4095.0f * 2.0f * M_PI));

  static int subloop_counter = 0;
  subloop_counter = (subloop_counter + 1) % 1000;

  if ((subloop_counter % 5) == 0) {
    // 50hz subloop
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    globals->led(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  }
  if ((subloop_counter % 10) == 0) {
    // 25hz subloop
    static PacketBuilder pb;
    static CustomControllerCmd cc_cmd;
    cc_cmd.ok = true;
    const auto joint_pos = globals->motors.GetPositions();
    std::memcpy(cc_cmd.joint_positions, joint_pos.data(), sizeof(float) * 6);
    cc_cmd.buttons.a = globals->buttons.a.is_set();
    cc_cmd.buttons.b = globals->buttons.b.is_set();
    cc_cmd.buttons.c = globals->buttons.c.is_set();
    cc_cmd.buttons.d = globals->buttons.d.is_set();
    cc_cmd.buttons.e = globals->buttons.e.is_set();
    const auto tx_pack = pb.Build(reinterpret_cast<const uint8_t *>(&cc_cmd), sizeof(cc_cmd));

    // 如果这一轮串口忙就跳过这一轮发送
    if (HAL_UART_GetState(&huart7) == HAL_UART_STATE_READY) {
      HAL_UART_Transmit_DMA(&huart7, tx_pack->data(), tx_pack->size());
    }
  }

  // 更新按钮状态
  globals->buttons.a.add(HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) == GPIO_PIN_RESET);
  globals->buttons.b.add(HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) == GPIO_PIN_RESET);
  globals->buttons.c.add(HAL_GPIO_ReadPin(KEY_C_GPIO_Port, KEY_C_Pin) == GPIO_PIN_RESET);
  globals->buttons.d.add(HAL_GPIO_ReadPin(KEY_D_GPIO_Port, KEY_D_Pin) == GPIO_PIN_RESET);
  globals->buttons.e.add(HAL_GPIO_ReadPin(KEY_E_GPIO_Port, KEY_E_Pin) == GPIO_PIN_RESET);

  if (g_comp_enable) {
    const auto joint_pos = globals->motors.GetPositions();
    const DynamicModel::Vec3 q_current = {0.f, joint_pos[1], joint_pos[2]};
    const DynamicModel::Vec3 g_comp = globals->dynamic_model.ForwardDynamics(q_current);
    if (subloop_counter % 2) {
      globals->motors.j0->SetSpeed(0);
      globals->motors.j1->SetMitCommand(0.f, 0.f, g_comp(1), 0.f, 1.f);
      globals->motors.j2->SetMitCommand(0.f, 0.f, -g_comp(2), 0.f, 0.5f);
    } else {
      globals->motors.j3->SetSpeed(0);
      globals->motors.j4->SetSpeed(0);
    }
  } else {
    if (subloop_counter % 2) {
      globals->motors.j0->SetPower(0);
      globals->motors.j1->SetMitCommand(0.f, 0.f, 0.f, 0.f, 0.f);
      globals->motors.j2->SetMitCommand(0.f, 0.f, 0.f, 0.f, 0.f);
    } else {
      globals->motors.j3->SetPower(0);
      globals->motors.j4->SetPower(0);
    }
  }

  /* ── 执行时间统计 ───────────────────────────────── */
  const uint32_t elapsed_us = DwtCycleToUs(DWT->CYCCNT - t_start);
  g_mainloop_time_avg.add(elapsed_us);
  g_mainloop_time_us = g_mainloop_time_avg.value();
}
}  // namespace

volatile uint32_t g_mainloop_time_us = 0;  // 供 UI 线程读取（单位 µs）

Globals *globals;

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ONBOARD_KEY_Pin) {
    g_comp_enable = !g_comp_enable;
    globals->g_comp_enable_watcher.Update(g_comp_enable);
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
  }
}

[[noreturn]] void AppMain(void) {
  DwtInit();  // 启动 DWT 周期计数器，用于 MainLoop 执行时间统计

  globals = new Globals;
  globals->Init();

  // 启动旋钮ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_adc1_val, 1);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);  // 关闭 DMA 中断，防止高频中断卡死 CPU

  /* ── LVGL initialization ─────────────────────────────── */
  lv_init();

  /* Use HAL_GetTick as the tick source (returns ms since boot) */
  lv_tick_set_cb(HAL_GetTick);

  /* Initialize the ST7789 display driver (SPI4 + DMA) */
  lv_display_t *disp = st7789_driver_init();
  (void)disp;

  /* Create the arm visualization UI */
  arm_ui::create();

  // // 设零点
  // HAL_Delay(1500);  // 电机开机需要时间
  // globals->motors.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  // globals->motors.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  // for (int i = 0; i < 10; i++) {
  //   // globals->motors.j1->SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
  //   // globals->motors.j2->SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
  //   // globals->motors.j0->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
  //   // globals->motors.j3->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
  //   globals->motors.j4->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
  //   // globals->motors.j5->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kSetCurrentAsEncoderZero);
  //   HAL_Delay(10);
  // }
  // globals->motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  // globals->motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);

  // 创建主循环定时任务，定频250hz
  TimerTask mainloop_250hz{
      &htim14,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_250hz.Start();

  globals->g_comp_enable_watcher.OnValueChange(
      etl::delegate<void(const bool &, const bool &)>::create([](const bool &old_value, const bool &new_value) {
        // 根据新值使能或失能电机（简单实现）
        if (new_value) {
          globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
          globals->motors.j0->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kEnable);
          globals->motors.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          globals->motors.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          globals->motors.j3->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kEnable);
          globals->motors.j4->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kEnable);
        } else {
          globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
          globals->motors.j0->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kDisable);
          globals->motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
          globals->motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
          globals->motors.j3->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kDisable);
          globals->motors.j4->SendInstruction(rm::device::LkMotorMS<15>::Instruction::kDisable);
        }
      }));

  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
    const uint32_t time_until_next = lv_timer_handler();
    HAL_Delay(time_until_next);
  }
}
}