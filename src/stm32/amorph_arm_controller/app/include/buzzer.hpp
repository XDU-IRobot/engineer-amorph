
#pragma once

#include <functional>

#include "tim.h"

/**
 * @brief   简单阻塞式蜂鸣器
 */
class Buzzer {
 public:
  Buzzer(TIM_HandleTypeDef& htim, uint32_t channel) : htim_(&htim), channel_(channel) {}

  void Init() { HAL_TIM_PWM_Start(htim_, channel_); }

  void Beep(int beeps, int interval_ms = 100) {
    for (int i = 0; i < beeps; ++i) {
      __HAL_TIM_SET_COMPARE(htim_, channel_, 500);  // Set duty cycle to 50%
      HAL_Delay(interval_ms);
      __HAL_TIM_SET_COMPARE(htim_, channel_, 0);  // Set duty cycle to 0%
      HAL_Delay(interval_ms);
    }
  }

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
};

/**
 * @brief   利用定时器中断实现的异步蜂鸣器
 */
class AsyncBuzzer {
 public:
  AsyncBuzzer(TIM_HandleTypeDef& htim, uint32_t channel, TIM_HandleTypeDef& htim_delay)
      : htim_(&htim), channel_(channel), htim_delay_(&htim_delay) {}

  void Init() {
    HAL_TIM_RegisterCallback(
        htim_delay_, HAL_TIM_PERIOD_ELAPSED_CB_ID,
        CallableObjToCallbackFnPtr(std::bind(&AsyncBuzzer::TimerCallback, this, std::placeholders::_1)));
    HAL_TIM_PWM_Start(htim_, channel_);
  }

  void Beep(int beeps) {
    if (beeps <= 0) {
      return;
    }
    beep_total_ = beeps * 2;  // 每次响和灭都算一次
    beep_count_ = 0;
    beep_on_ = false;
    HAL_TIM_Base_Start_IT(htim_delay_);  // 启动定时器中断
  }

 private:
  static pTIM_CallbackTypeDef CallableObjToCallbackFnPtr(std::function<void(TIM_HandleTypeDef*)> func) {
    static auto fn = std::move(func);
    return [](TIM_HandleTypeDef* htim) { fn(htim); };
  }

  void TimerCallback(TIM_HandleTypeDef* htim) {
    if (beep_count_ < beep_total_) {
      if (beep_on_) {
        __HAL_TIM_SET_COMPARE(htim_, channel_, 0);  // Set duty cycle to 0%
      } else {
        __HAL_TIM_SET_COMPARE(htim_, channel_, 500);  // Set duty cycle to 50%
      }
      beep_on_ = !beep_on_;
      beep_count_++;
    } else {
      __HAL_TIM_SET_COMPARE(htim_, channel_, 0);  // Set duty cycle to 0%
      HAL_TIM_Base_Stop_IT(htim_);
      beep_count_ = 0;
      beep_total_ = 0;
      beep_on_ = false;
    }
  }

 private:
  int beep_count_{0};
  int beep_total_{0};
  bool beep_on_{false};

  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
  TIM_HandleTypeDef* htim_delay_;
};