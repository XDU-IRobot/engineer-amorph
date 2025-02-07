
/**
 * @file  gpio_mock.hpp
 * @brief 这个文件是一个wiringpi库的mock，为了在没有wiringpi的环境下通过编译，不要直接include这个文件！
 */

#ifndef ARM_GPIO_MOCK_HPP
#define ARM_GPIO_MOCK_HPP

#include "common/logger.hpp"

constexpr int OUTPUT = 0;
constexpr int PWM_OUTPUT = 1;
constexpr int LOW = 0;
constexpr int HIGH = 1;

class OutputPin {
 public:
  explicit OutputPin(int pin_num) : pin_num_{pin_num} {}

  void Write(int value) {
    info("Write to pin %d: %d -> %d", pin_num_, state_, value);
    state_ = value;
  }
  bool state() const { return state_; }

 private:
  int pin_num_;
  bool state_;
};

class PwmOutputPin {
 public:
  explicit PwmOutputPin(int pin_num) : pin_num_{pin_num} {}

  void Write(double duty) {
    std::clamp(duty, 0.0, 1.0);
    info("Write duty to pwm pin %d: %f -> %f", pin_num_, duty_, duty);
    duty_ = duty;
  }
  double duty() const { return duty_; }

 private:
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
