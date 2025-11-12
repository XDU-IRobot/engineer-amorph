
#ifndef ARM_GPIO_HPP
#define ARM_GPIO_HPP

#include <algorithm>

#include <wiringPi.h>
#include <softPwm.h>

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
    state_ = static_cast<bool>(value);
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
    softPwmCreate(pin_num, 0, kResolution);
    // pinMode(pin_num, PWM_OUTPUT);
    // pwmSetRange(pin_num, kResolution);
    // pwmSetClock(pin_num, 480);
    // pwmWrite(pin_num, 0);
  }

  void Write(double duty) {
    std::clamp(duty, 0.0, 1.0);
    duty = 1.0 - duty;
    softPwmWrite(pin_num_, duty * kResolution);
    duty_ = duty;
  }
  double duty() const { return duty_; }

 private:
  constexpr static int kResolution = 200;

  int pin_num_;
  double duty_;
};

template <int PinMode>
auto Pin(int pin_num) {
  if constexpr (PinMode == OUTPUT) {
    return OutputPin{pin_num};
  } else if constexpr (PinMode == PWM_OUTPUT) {
    return PwmOutputPin{pin_num};
  }
}

#endif