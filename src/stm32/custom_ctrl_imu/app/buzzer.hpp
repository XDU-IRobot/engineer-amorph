#pragma once

#include <functional>

#include "tim.h"

/**
 * @brief   简单蜂鸣器
 */
class Buzzer {
 public:
  explicit Buzzer(TIM_HandleTypeDef& htim = htim4, uint32_t channel = TIM_CHANNEL_3);

  void Init() const;

  void SetFrequency(uint32_t frequency) const;

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
};
