
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"

#include <librm.hpp>

#include "misc.hpp"
#include "bsp_led.h"

extern "C" {

[[noreturn]] void AppMain(void) {
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
  BuzzerBeep(2);
  aRGB_led_show(0xff00ff00);

  hal::Serial *dbus = new hal::Serial{huart3, 36, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};
  device::BMI088 *imu = new device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  device::DR16 *rc = new device::DR16{*dbus};
  rc->Begin();

  for (;;) {
    imu->Update();
    aRGB_led_show(0xff00ff00);
    HAL_Delay(500);
    aRGB_led_show(0x00000000);
    HAL_Delay(500);
  }
}
}