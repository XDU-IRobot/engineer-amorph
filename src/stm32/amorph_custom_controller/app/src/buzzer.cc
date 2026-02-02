
#include "buzzer.hpp"

Buzzer::Buzzer(TIM_HandleTypeDef &htim, uint32_t channel, uint32_t tim_clk_freq)
    : htim_(&htim), channel_(channel), tim_clk_freq_(tim_clk_freq) {}

void Buzzer::Init() { HAL_TIM_PWM_Start(htim_, channel_); }

void Buzzer::SetFrequency(uint32_t frequency) {
  if (frequency <= 0) {
    __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
    return;
  }

  // 搜索合适的PSC和ARR组合
  // PSC和ARR都是16位值，范围是0-65535
  // 实际分频系数 = (PSC + 1)
  // 实际周期 = (ARR + 1)
  // PWM频率 = tim_clk_freq / ((PSC + 1) * (ARR + 1))
  // 目标: (PSC + 1) * (ARR + 1) = tim_clk_freq / frequency

  uint32_t target_period = tim_clk_freq_ / frequency;
  uint16_t best_psc = 0;
  uint16_t best_arr = 0;
  uint32_t min_error = 0xFFFFFFFF;

  // 从较小的分频开始搜索，优先使用更高的ARR值以获得更好的分辨率
  for (uint32_t psc = 0; psc <= 65535; psc++) {
    uint32_t arr = target_period / (psc + 1) - 1;

    // ARR必须在有效范围内
    if (arr > 65535) continue;
    if (arr < 1) break;  // 后续的PSC只会让ARR更小，可以提前退出

    // 计算实际频率的误差
    uint32_t actual_period = (psc + 1) * (arr + 1);
    uint32_t error = (actual_period > target_period) ?
                     (actual_period - target_period) :
                     (target_period - actual_period);

    if (error < min_error) {
      min_error = error;
      best_psc = psc;
      best_arr = arr;

      // 如果找到精确匹配，提前退出
      if (error == 0) break;
    }
  }

  // 设置定时器参数
  __HAL_TIM_SET_PRESCALER(htim_, best_psc);
  __HAL_TIM_SET_AUTORELOAD(htim_, best_arr);
  __HAL_TIM_SET_COMPARE(htim_, channel_, best_arr / 2);  // 50%占空比
}