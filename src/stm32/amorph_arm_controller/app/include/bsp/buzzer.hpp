
#pragma once

#include "tim.h"

/**
 * @brief   无源蜂鸣器驱动
 */
class Buzzer {
 public:
  Buzzer(TIM_HandleTypeDef& htim, uint32_t channel) : htim_(&htim), channel_(channel) {}

  void Init() { HAL_TIM_PWM_Start(htim_, channel_); }

  void SetFrequency(uint32_t frequency) {
    if (frequency == 0) {
      __HAL_TIM_SET_COMPARE(htim_, channel_, 0);  // 关闭蜂鸣器
      return;
    }
    frequency *= 2;
    __HAL_TIM_SET_AUTORELOAD(htim_, 4000000 / frequency - 1);
    __HAL_TIM_SET_COMPARE(htim_, channel_, (4000000 / frequency) / 2);  // 50%占空比
  }

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
};
