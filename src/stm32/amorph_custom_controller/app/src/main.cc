
#include <librm.hpp>

#include "buzzer.hpp"
#include "globals.hpp"
#include "packet_builder.hpp"
#include "timer_task.hpp"

Globals globals;

namespace {

bool g_comp_enable = false;

void MainLoop() {
  static int subloop_counter = 0;
  subloop_counter = (subloop_counter + 1) % 1000;

  if ((subloop_counter % 5) == 0) {
    // 50hz subloop
    globals.buzzer.SetFrequency(globals.buzzer_controller.Update().frequency);
    const auto &[led_r, led_g, led_b] = globals.led_controller.Update();
    globals.led(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  }
  if ((subloop_counter % 10) == 0) {
    // 25hz subloop
    static PacketBuilder pb;
    const float joint_pos[6] = {globals.motors.j0->feedback().position_rad,  //
                                globals.motors.j1->pos(),                    //
                                globals.motors.j2->pos(),                    //
                                globals.motors.j0->feedback().position_rad,  //
                                globals.motors.j0->feedback().position_rad,  //
                                globals.motors.j0->feedback().position_rad};
    static_assert(sizeof(joint_pos) <= 30);
    const auto tx_pack = pb.Build(reinterpret_cast<const uint8_t *>(&joint_pos), sizeof(joint_pos));

    // 等待串口空闲，如果这一轮串口忙就跳过这一轮发送
    if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY) {
      HAL_UART_Transmit_IT(&huart6, tx_pack->data(), tx_pack->size());
    }
  }

  globals.imu->Update();
  globals.ahrs.Update(rm::modules::ImuData6Dof{globals.imu->gyro_x(), globals.imu->gyro_y(), globals.imu->gyro_z(),
                                               globals.imu->accel_x(), globals.imu->accel_x(), globals.imu->accel_x()});

  static bool all_motors_online = true;  // 第一次假设所有电机在线，不然进不了后续逻辑
  if (!all_motors_online) {
    globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    return;  // 有电机离线则不运行后续逻辑
  }

  if (g_comp_enable) {
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
}  // namespace

extern "C" {

[[noreturn]] void AppMain(void) {
  globals.Init();

  // 创建主循环定时任务，定频250hz
  TimerTask mainloop_250hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_250hz.SetPrescalerAndPeriod(168 - 1, 2000 - 1);
  mainloop_250hz.Start();

  globals.g_comp_enable_watcher.OnValueChange(
      etl::delegate<void(const bool &, const bool &)>::create([](const bool &old_value, const bool &new_value) {
        // 根据新值使能或失能电机（简单实现）
        if (new_value) {
          globals.led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
          globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        } else {
          globals.led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
          globals.motors.j1->SendInstruction(rm::device::DmMotorInstructions::kDisable);
          globals.motors.j2->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        }
      }));

  for (;;) {
    // __WFI();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ONBOARD_KEY_Pin) {
    g_comp_enable = !g_comp_enable;
    globals.g_comp_enable_watcher.Update(g_comp_enable);
    globals.buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
  }
}
}