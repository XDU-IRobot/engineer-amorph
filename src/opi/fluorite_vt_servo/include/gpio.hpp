
#ifndef ARM_GPIO_HPP
#define ARM_GPIO_HPP

#include <algorithm>

#if __has_include("wiringPi.h")

#include <wiringPi.h>

static bool wiringpi_initiallized = false;

inline void InitWiringPi() {
  if (!wiringpi_initiallized) {
    wiringPiSetup();
    wiringpi_initiallized = true;
  }
}

class OutputPin {
 public:
  explicit OutputPin(int pin_num) : pin_num_{pin_num} {
    InitWiringPi();
    pinMode(pin_num, OUTPUT);
  }

  void Write(int value) {
    digitalWrite(pin_num_, value);
    state_ = value;
  }
  bool state() const { return state_; }

 private:
  int pin_num_;
  bool state_;
};

class PwmOutputPin {
 public:
  explicit PwmOutputPin(int pin_num) : pin_num_{pin_num} {
    InitWiringPi();
    pinMode(pin_num, PWM_OUTPUT);
    pwmSetRange(pin_num, kResolution);
    pwmWrite(pin_num, 0);
  }

  void Write(double duty) {
    std::clamp(duty, 0.0, 1.0);
    pwmWrite(pin_num_, duty * kResolution);
    duty_ = duty;
  }
  bool duty() const { return duty_; }

 private:
  constexpr static int kResolution = 1024;

  int pin_num_;
  bool duty_;
};

template <int PinMode>
auto Pin(int pin_num) {
  if constexpr (PinMode == OUTPUT) {
    return OutputPin{pin_num};
  } else if constexpr (PinMode == PWM_OUTPUT) {
    return PwmOutputPin{pin_num};
  }
}

#else

#include "gpio_mock.hpp"

#endif

#endif