
#include <librm.hpp>

#include "main.h"

#include "can.h"
#include "spi.h"

#include "buzzer.hpp"
#include "led.hpp"
#include "timer_task.hpp"
#include "schema.hpp"

struct Globals {
  Buzzer buzzer;
  LED led;
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Startup> buzzer_controller;
  rm::modules::RgbLedController<rm::modules::led_pattern::GreenBreath> led_controller;

  rm::hal::Can can2{hcan2};
  rm::device::BMI088 imu{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

  rm::modules::MahonyAhrs ahrs{1000.f};

  void Init() {
    can2.SetFilter(0, 0);
    can2.Begin();
    buzzer.Init();
    led.Init();
  }
} *globals;

namespace {
void MainLoop1khz() {
  globals->imu.Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{.gx = globals->imu.gyro_x() - 0.001316666867584f,
                                                .gy = globals->imu.gyro_y() + 0.0007636880887672f,
                                                .gz = globals->imu.gyro_z() - 0.0010046509085223f,
                                                .ax = globals->imu.accel_x(),
                                                .ay = globals->imu.accel_y(),
                                                .az = globals->imu.accel_z()});

  schema::EulerAngle pack;
  pack.pitch = rm::modules::F32ToF16(globals->ahrs.euler_angle().pitch);
  pack.roll = rm::modules::F32ToF16(globals->ahrs.euler_angle().roll);
  pack.yaw = rm::modules::F32ToF16(globals->ahrs.euler_angle().yaw);
  globals->can2.Write(schema::EulerAngle::kFrameId, (uint8_t *)&pack, sizeof(pack));

  static size_t prescaler = 0;
  if (prescaler % 10 == 0) {
    const auto [r, g, b] = globals->led_controller.Update();
    globals->led(0xff000000 | r << 16 | g << 8 | b);
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
  }

  prescaler = (prescaler + 1) % 1000;
}
}  // namespace

extern "C" {
void AppMain(void) {
  globals = new Globals;
  globals->Init();

  // HAL_Delay(1000);
  // static double gyro_bias[3]{};
  // for (int i = 0; i < 10000; ++i) {
  //   globals->imu.Update();
  //   gyro_bias[0] += globals->imu.gyro_x() / 10000.0;
  //   gyro_bias[1] += globals->imu.gyro_y() / 10000.0;
  //   gyro_bias[2] += globals->imu.gyro_z() / 10000.0;
  // }

  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                       //
      etl::delegate<void()>::create<MainLoop1khz>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();

  for (;;) {
  }
}
}