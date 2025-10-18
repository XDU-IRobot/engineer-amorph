
#include "misc.hpp"

#include "tim.h"

// 异步蜂鸣器相关静态变量
static int async_beep_count = 0;
static int async_beep_total = 0;
static bool async_beep_on = false;
/**
 * @brief   异步蜂鸣器响指定次数（利用TIM14中断）
 * @param   beeps 哔哔声次数
 */
void AsyncBuzzerBeep(int beeps) {
  if (beeps <= 0) {
    return;
  }
  async_beep_total = beeps * 2;  // 每次响和灭都算一次
  async_beep_count = 0;
  async_beep_on = false;
  HAL_TIM_Base_Start_IT(&htim14);  // 启动TIM14中断
}

/**
 * @brief   蜂鸣器发出指定次数的哔哔声
 * @param   beeps 哔哔声次数
 */
void BuzzerBeep(int beeps) {
  for (int i = 0; i < beeps; ++i) {
    TIM4->CCR3 = 20999 / 2;
    HAL_Delay(50);
    TIM4->CCR3 = 0;
    HAL_Delay(50);
  }
}

void Laser(bool on) {
  static bool tim3_pwm_started = false;
  if (!tim3_pwm_started) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  }
  if (on) {
    TIM3->CCR3 = 0;
  } else {
    TIM3->CCR3 = 8399;
  }
}

// TIM14中断回调函数（需在irq_handlers.cc调用）
extern "C" void AsyncBuzzerBeep_TIM14_IRQHandler() {
  if (async_beep_count < async_beep_total) {
    if (async_beep_on) {
      TIM4->CCR3 = 0;
    } else {
      TIM4->CCR3 = 20999 / 2;
    }
    async_beep_on = !async_beep_on;
    async_beep_count++;
  } else {
    TIM4->CCR3 = 0;
    HAL_TIM_Base_Stop_IT(&htim14);
    async_beep_count = 0;
    async_beep_total = 0;
    async_beep_on = false;
  }
}