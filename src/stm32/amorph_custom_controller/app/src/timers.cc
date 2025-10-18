
#include "tim.h"

#include "globals.hpp"
#include "buzzer.hpp"  // AsyncBuzzerBeep_TIM14_IRQHandler

/**
 * @brief   定频1khz运行姿态解算
 */
void TIM10_1000hz() {
  globals.imu->Update();
  globals.ahrs.Update(modules::ImuData6Dof{globals.imu->gyro_x(), globals.imu->gyro_y(), globals.imu->gyro_z(),
                                           globals.imu->accel_x(), globals.imu->accel_x(), globals.imu->accel_x()});
}

/**
 * @brief   定频500hz运行主循环
 */
void TIM13_500hz() {}

extern "C" {
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM10) [[likely]] {
    TIM10_1000hz();
  } else if (htim->Instance == TIM13) {
    TIM13_500hz();
  } else if (htim->Instance == TIM14) {
    AsyncBuzzerBeep_TIM14_IRQHandler();
  }
}
}