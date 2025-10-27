
#include "tim.h"

#include "globals.hpp"
#include "buzzer.hpp"  // AsyncBuzzerBeep_TIM14_IRQHandler

/**
 * @brief   定频1khz运行姿态解算
 */
void TIM10_1000hz() {
  globals.imu->Update();
  globals.ahrs.Update(rm::modules::ImuData6Dof{globals.imu->gyro_x(), globals.imu->gyro_y(), globals.imu->gyro_z(),
                                               globals.imu->accel_x(), globals.imu->accel_x(), globals.imu->accel_x()});
}

bool run_g_comp = false;  ///< 是否运行重力补偿，按键切换，切换逻辑在下面的外部中断回调里
bool previous_run_g_comp = false;

/**
 * @brief   定频500hz运行主循环
 */
void TIM13_500hz() {
  static bool all_motors_online = true;  // 第一次假设所有电机在线，不然进不了后续逻辑
  if (!all_motors_online) {
    globals.led(0xffffff00);  // 黄灯代表有电机离线
    return;                   // 有电机离线则不运行后续逻辑
  }

  if (previous_run_g_comp != run_g_comp) {
    AsyncBuzzerBeep(1);
    globals.led(run_g_comp ? 0xff00ff00 : 0xffff0000);  // 绿灯代表重力补偿开启，红灯代表关闭
    if (run_g_comp) {
      // 开启重力补偿，使能电机
      globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    } else {
      // 关闭重力补偿，失能电机
      globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
      globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    HAL_Delay(1);  // 防止一次发太多报文把CAN mailbox塞满
  }
  previous_run_g_comp = run_g_comp;

  if (run_g_comp) {
    const DynamicModel::Vec3 q_current = {0.f, globals.motors.j1->pos(), globals.motors.j2->pos()};
    const DynamicModel::Vec3 g_comp = globals.dynamic_model.ForwardDynamics(q_current);
    globals.motors.j1->SetPosition(0.f, 0.f, g_comp(1), 0.f, 0.f);
    globals.motors.j2->SetPosition(0.f, 0.f, -g_comp(2), 0.f, 0.f);
  } else {
    globals.motors.j1->SetPosition(0.f, 0.f, 0.f, 0.f, 0.f);
    globals.motors.j2->SetPosition(0.f, 0.f, 0.f, 0.f, 0.f);
  }

  // all_motors_online = globals.motors.j1->IsAlive() && globals.motors.j2->IsAlive();  // 更新电机在线状态
}

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ONBOARD_KEY_Pin) {
    run_g_comp = !run_g_comp;
  }
}
}