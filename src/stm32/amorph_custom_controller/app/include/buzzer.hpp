
#pragma once

#include "tim.h"

/**
 * @brief   简单蜂鸣器
 */
class Buzzer {
 public:
  explicit Buzzer(TIM_HandleTypeDef& htim = htim12, uint32_t channel = TIM_CHANNEL_1, uint32_t tim_clk_freq = 84000000);

  void Init();

  void SetFrequency(uint32_t frequency);

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
  uint32_t tim_clk_freq_;  // 定时器时钟频率
};