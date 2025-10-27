
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

float g_comp_arr[3] = {0.f, 0.f, 0.f};

/**
 * @brief   定频500hz运行主循环
 */
void TIM13_500hz() {
  static rm::device::DR16::SwitchPosition previous_switch_r = rm::device::DR16::SwitchPosition::kUnknown;

  if (!globals.rc->IsAlive()) {
    globals.led(0xffff0000);
    return;
  } else {
    globals.led(0xff00ff00);
  }

  if (previous_switch_r != globals.rc->switch_r()) {
    AsyncBuzzerBeep(1);
    switch (globals.rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp: {
        globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        HAL_Delay(1);
        break;
      }
      default: {
        globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        HAL_Delay(1);
        break;
      }
    }
    previous_switch_r = globals.rc->switch_r();
  }

  const DynamicModel::Vec3 q_current = {0.f, globals.motors.j1->pos(), globals.motors.j2->pos()};
  const DynamicModel::Vec3 g_comp = globals.dynamic_model.ForwardDynamics(q_current);
  g_comp_arr[0] = g_comp(0);
  g_comp_arr[1] = g_comp(1);
  g_comp_arr[2] = g_comp(2);
  if (globals.rc->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    globals.motors.j1->SetPosition(0.f, 0.f, g_comp(1), 0.f, 0.f);
    globals.motors.j2->SetPosition(0.f, 0.f, -g_comp(2), 0.f, 0.f);
  }
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
}