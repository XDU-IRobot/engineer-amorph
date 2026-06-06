#pragma once

#include <librm.hpp>
#include <etl/debounce.h>

#include "gpio.h"

class Buttons {
 public:
  enum { kOnBoardKey, kA, kB, kC, kD, kE };

  struct Event {
    int button_idx;
    bool pressed;
  };

  using Callback = std::function<void(const Event&)>;

  void Update() {
    raw_.a = HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin) == GPIO_PIN_RESET;
    raw_.b = HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin) == GPIO_PIN_RESET;
    raw_.c = HAL_GPIO_ReadPin(KEY_C_GPIO_Port, KEY_C_Pin) == GPIO_PIN_RESET;
    raw_.d = HAL_GPIO_ReadPin(KEY_D_GPIO_Port, KEY_D_Pin) == GPIO_PIN_RESET;
    raw_.e = HAL_GPIO_ReadPin(KEY_E_GPIO_Port, KEY_E_Pin) == GPIO_PIN_RESET;
    a_.add(raw_.a);
    b_.add(raw_.b);
    c_.add(raw_.c);
    d_.add(raw_.d);
    e_.add(raw_.e);
  }

  bool a() const { return a_.is_set(); }
  bool b() const { return b_.is_set(); }
  bool c() const { return c_.is_set(); }
  bool d() const { return d_.is_set(); }
  bool e() const { return e_.is_set(); }

 private:
  struct {
    bool a, b, c, d, e;
  } raw_;
  etl::debounce<10, 10, 10> a_, b_, c_, d_, e_;
};