#pragma once

#include "gpio.h"

class LED {
 public:
  void Init() {
    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  }

  void operator()(uint32_t argb) {
    static uint8_t alpha;
    static uint16_t red, green, blue;

    alpha = (argb & 0xFF000000) >> 24;
    red = ((argb & 0x00FF0000) >> 16) * alpha;
    green = ((argb & 0x0000FF00) >> 8) * alpha;
    blue = ((argb & 0x000000FF) >> 0) * alpha;

    if (red > 127) {
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    }

    if (green > 127) {
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    }
  }
};