
#include <etl/cyclic_value.h>

#include "tim.h"

#include "globals.hpp"

/**
 * @brief   定频1khz运行主逻辑
 */
void TIM13_1000hz() {
  static etl::cyclic_value<int, 0, 100> prescale_10hz;

  // 10hz维护RPC连接
  ++prescale_10hz;
  if (prescale_10hz.get() == 0) {
    globals->rpc.KeepConnection();
    globals->rpc.TickTimer();
  }
}

extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM13) [[likely]] {
    TIM13_1000hz();
  }
}
}