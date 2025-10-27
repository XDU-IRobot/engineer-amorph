
#include "tim.h"

#include "globals.hpp"

Globals *globals{nullptr};

extern "C" {

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();
  HAL_TIM_Base_Start_IT(&htim13);  // 启动1kHz定时器，开始运行主逻辑循环

  for (;;) {
    __WFI();
  }
}
}