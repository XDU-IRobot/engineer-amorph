
#ifndef ARM_GPIO_HPP
#define ARM_GPIO_HPP

#include <algorithm>

#if __has_include("wiringPi.h")

#include <wiringPi.h>

class OutputPin {
 public:
  explicit OutputPin(int pin_num) : pin_num_{pin_num} { pinMode(pin_num, OUTPUT); }

  void Write(int value) {
    digitalWrite(pin_num_, value);
    state_ = value;
  }
  bool state() const { return state_; }

 private:
  int pin_num_;
  bool state_;
};

/**
 * @brief wiringop的pwm有很严重的bug，所以这个pwm类暂时和outputpin一样，大于0.5就是HIGH，小于等于0.5就是LOW
 *
 */
class PwmOutputPin {
 public:
  explicit PwmOutputPin(int pin_num, size_t frequency = 50)
      : pin_num_{pin_num}, frequency_(frequency), duty_(0.0), stop_emulation_(false) {
    pinMode(pin_num, OUTPUT);  // 设置引脚为输出模式
    StartEmulation();          // 启动模拟 PWM 的线程
  }

  ~PwmOutputPin() {
    stop_emulation_ = true;  // 停止线程
    if (emulation_thread_.joinable()) {
      emulation_thread_.join();
    }
  }

  void Write(double duty) {
    duty_ = std::clamp(duty, 0.0, 1.0);  // 限制占空比在 0 到 1 之间
  }

  double duty() const { return duty_; }

 private:
  void StartEmulation() {
    emulation_thread_ = std::thread([this]() {
      const auto period = std::chrono::microseconds(static_cast<int>(1e6 / frequency_));
      while (!stop_emulation_) {
        if (duty_ > 0.0) {
          digitalWrite(pin_num_, HIGH);                 // 设置引脚为高电平
          std::this_thread::sleep_for(period * duty_);  // 持续高电平时间
        }
        if (duty_ < 1.0) {
          digitalWrite(pin_num_, LOW);                          // 设置引脚为低电平
          std::this_thread::sleep_for(period * (1.0 - duty_));  // 持续低电平时间
        }
      }
    });
  }

  std::thread emulation_thread_;
  const int pin_num_;
  const size_t frequency_;
  double duty_;
  bool stop_emulation_;
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