
#include "tim.h"

#include <librm.hpp>

#include "buzzer.hpp"
#include "globals.hpp"

extern "C" {

[[noreturn]] void AppMain(void) {
  globals.Init();

  // HAL_TIM_Base_Start_IT(&htim10);  // 启动姿态解算
  // HAL_Delay(2);                    // 等待姿态收敛

  HAL_TIM_Base_Start_IT(&htim13);  // 启动主循环
  AsyncBuzzerBeep(2);

  for (;;) {
    __WFI();
  }
}
}