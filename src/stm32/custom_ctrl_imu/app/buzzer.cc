#include "buzzer.hpp"

Buzzer::Buzzer(TIM_HandleTypeDef &htim, uint32_t channel) : htim_(&htim), channel_(channel) {}

void Buzzer::Init() const { HAL_TIM_PWM_Start(htim_, channel_); }

void Buzzer::SetFrequency(uint32_t frequency) const {
  if (frequency <= 0) {
    __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
    return;
  }
  const uint32_t period = 84000000 / (frequency * 2);
  __HAL_TIM_SET_AUTORELOAD(htim_, period);
  __HAL_TIM_SET_COMPARE(htim_, channel_, period / 2);
}
