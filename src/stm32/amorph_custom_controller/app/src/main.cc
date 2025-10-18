
#include "main.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"

#include <librm.hpp>

#include "buzzer.hpp"
#include "globals.hpp"

void SpawnGlobalObjects() {
  globals.dbus = new hal::Serial{huart3, 36, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};
  globals.imu = new device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  globals.rc = new device::DR16{*globals.dbus};
  globals.motor_can = new hal::Can{hcan1};
  for (size_t i = 0; i < 6; ++i) {
    globals.motors[i] = new device::LkMotor<device::LkMotorType::kMS, 15>{*globals.motor_can, i + 1};
  }
  globals.rc->Begin();
  globals.motor_can->Begin();
  globals.led.Init();
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);  // buzzer
};

extern "C" {

[[noreturn]] void AppMain(void) {
  SpawnGlobalObjects();
  globals.led(0xffffff00);

  HAL_TIM_Base_Start_IT(&htim10);  // 启动姿态解算
  HAL_Delay(2);                    // 等待姿态收敛

  HAL_TIM_Base_Start_IT(&htim13);  // 启动主循环
  AsyncBuzzerBeep(2);
  globals.led(0xff00ff00);

  for (;;) {
    HAL_Delay(HAL_MAX_DELAY);
  }
}
}