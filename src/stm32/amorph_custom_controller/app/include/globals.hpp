
#pragma once

#include <librm.hpp>

#include "dynamics.hpp"
#include "rgb_led.hpp"

struct Globals {
  device::BMI088 *imu;
  modules::MahonyAhrs ahrs;

  hal::Serial *dbus;
  device::DR16 *rc;

  hal::Can *motor_can;
  device::LkMotor<device::LkMotorType::kMS, 15> *motors[6];
  DynamicModel dynamic_model;
  LED led;
};

extern Globals globals;