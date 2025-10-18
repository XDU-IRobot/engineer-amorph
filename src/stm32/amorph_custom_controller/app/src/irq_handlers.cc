
#include "tim.h"

#include "misc.hpp"  // AsyncBuzzerBeep_TIM14_IRQHandler

/**
 * @brief   定频1khz运行主循环
 */
void TIM10_1000hz() {}

/**
 * @brief   定频500hz运行主循环
 */
void TIM13_500hz() {}

extern "C" {
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//   if (htim->Instance == TIM10) [[likely]] {
//     TIM10_1000hz();
//   } else if (htim->Instance == TIM13) {
//     TIM13_500hz();
//   } else if (htim->Instance == TIM14) {
//     AsyncBuzzerBeep_TIM14_IRQHandler();
//   }
// }
}