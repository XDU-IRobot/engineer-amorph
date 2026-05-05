#pragma once

#include <etl/debounce.h>

#include "gpio.h"

class Buttons {
 public:
  enum { kOnBoardKey, kA, kB, kC, kD, kE };

  void Update() {
    a_.add(HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) == GPIO_PIN_RESET);
    b_.add(HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) == GPIO_PIN_RESET);
    c_.add(HAL_GPIO_ReadPin(KEY_C_GPIO_Port, KEY_C_Pin) == GPIO_PIN_RESET);
    d_.add(HAL_GPIO_ReadPin(KEY_D_GPIO_Port, KEY_D_Pin) == GPIO_PIN_RESET);
    e_.add(HAL_GPIO_ReadPin(KEY_E_GPIO_Port, KEY_E_Pin) == GPIO_PIN_RESET);
  }

  bool a() const { return a_.is_set(); }
  bool b() const { return b_.is_set(); }
  bool c() const { return c_.is_set(); }
  bool d() const { return d_.is_set(); }
  bool e() const { return e_.is_set(); }

 private:
  etl::debounce<10, 10, 10> a_, b_, c_, d_, e_;
};