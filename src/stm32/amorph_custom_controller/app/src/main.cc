
#include <librm.hpp>

#include "usart.h"
#include "usbd_cdc_if.h"

#include "fsm.hpp"
// #include "arm_ui.hpp"
#include "buzzer.hpp"
#include "globals.hpp"
#include "packet_builder.hpp"
#include "timer_task.hpp"
#include "custom_controller_schema.hpp"

volatile long long g_mainloop_time_us = 0;

namespace {

void MainLoop() {
  const auto start = std::chrono::steady_clock::now();

  static int subloop_counter = 0;
  subloop_counter = (subloop_counter + 1) % 1000;

  // 50hz subloop 慢IO
  if ((subloop_counter % 5) == 0) {
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    // globals->led(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  }

  // 25hz subloop 串口发送
  if ((subloop_counter % 10) == 0) {
#if defined(I_AM_LEFT_CC)  // slave
    static SlaveData fb_data;
    fb_data.ok = globals->actuator.dev_manager_.all_device_ok();
    fb_data.btn_a = globals->buttons.a();
    fb_data.btn_b = globals->buttons.b();
    fb_data.btn_c = globals->buttons.c();
    const auto joint_pos = globals->actuator.GetPositions();
    for (int i = 0; i < 6; ++i) {
      fb_data.q[i] = joint_pos[i];
    }
    globals->ms.ReportToMaster(fb_data);
#elif defined(I_AM_RIGHT_CC)  // master
    // globals->servo.MoveTime(HAL_GetTick() % 1000, 1.f / 25.f);s

    static PacketBuilder pb;
    static CustomControllerCommand cc_cmd;
    cc_cmd.ok = globals->actuator.dev_manager_.all_device_ok() &&            //
                (globals->ms.online_status() == rm::device::Device::kOk) &&  //
                globals->ms.data().ok;
    const auto joint_pos = globals->actuator.GetPositions();
    const auto &slave_data = globals->ms.data();
    for (int i = 0; i < 6; ++i) {
      cc_cmd.q_right[i] = joint_pos[i];
      cc_cmd.q_left[i] = slave_data.q[i];
    }
    cc_cmd.left_btn_a = slave_data.btn_a;
    cc_cmd.left_btn_b = slave_data.btn_b;
    cc_cmd.left_btn_c = slave_data.btn_c;
    cc_cmd.right_btn_a = globals->buttons.a();
    cc_cmd.right_btn_b = globals->buttons.b();
    cc_cmd.right_btn_c = globals->buttons.c();
    // cc_cmd.buttons.d = globals->buttons.d();
    const auto compressed = cc_cmd.ToCompressed();
    const auto tx_pack = pb.Build(reinterpret_cast<const uint8_t *>(&compressed), sizeof(compressed));

    // 如果这一轮串口忙就跳过这一轮发送
    if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY) {
      HAL_UART_Transmit_DMA(&huart6, tx_pack->data(), tx_pack->size());
    }
#endif

    // // 向USB发送关节位置
    // // stamp,q1,q2,q3,dq1,dq2,dq3,tau1,tau2,tau3
    // char str[256];
    // std::sprintf(str, "%f,%.6f,%.6f,%.6f,0,0,0,0,%.6f,%.6f\r\n", static_cast<float>(HAL_GetTick()), joint_pos[0],
    //              joint_pos[1], joint_pos[2], -globals->actuator.j2.torque(), globals->actuator.j3.tau());
    // CDC_Transmit_FS(reinterpret_cast<uint8_t *>(str), std::strlen(str));
  }

  // 更新按钮状态、更新设备状态
  globals->buttons.Update();
  globals->actuator.dev_manager_.Update();

  // 触发控制循环
  fsm::cc_arm.receive(fsm::event::ControlLoop{});

  g_mainloop_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

Globals *globals;

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ONBOARD_KEY_Pin) {
    fsm::cc_arm.receive(fsm::event::ButtonEvent{Buttons::kOnBoardKey});
  }
}

[[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();

  fsm::Init();

  // // 启动旋钮ADC
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_adc1_val, 1);
  // __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);  // 关闭 DMA 中断，防止高频中断卡死 CPU

  // /* ── LVGL initialization ─────────────────────────────── */
  // lv_init();
  //
  // /* Use HAL_GetTick as the tick source (returns ms since boot) */
  // lv_tick_set_cb(HAL_GetTick);
  //
  // /* Initialize the ST7789 display driver (SPI4 + DMA) */
  // lv_display_t *disp = st7789_driver_init();
  // (void)disp;
  //
  // /* Create the arm visualization UI */
  // arm_ui::create();

  // // 设零点
  // HAL_Delay(3000);  // 电机开机需要时间
  // globals->actuator.SetZero();

#if defined(I_AM_RIGHT_CC)
  globals->ms.MasterStartRx();
#endif

  // 创建主循环定时任务，定频250hz
  TimerTask mainloop_250hz{
      &htim14,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };

  mainloop_250hz.Start();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
    globals->motor_can.Process();
  }
}
}