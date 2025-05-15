
/**
 * @file  gpio_mock.hpp
 * @brief 这个文件是一个wiringpi库的mock，为了在没有wiringpi的环境下通过编译，不要直接include这个文件！
 */

#ifndef ARM_GPIO_MOCK_HPP
#define ARM_GPIO_MOCK_HPP

#include <iostream>
#include <algorithm>
#include <thread>

constexpr int OUTPUT = 0;
constexpr int PWM_OUTPUT = 1;
constexpr int LOW = 0;
constexpr int HIGH = 1;

inline void wiringPiSetup() { std::printf("wiringPiSetup\n"); }

class OutputPin {
 public:
  inline explicit OutputPin(int pin_num) : pin_num_{pin_num} {}

  inline void Write(int value) {
    std::printf("Write to pin %d: %d -> %d\n", pin_num_, state_, value);
    state_ = value;
  }
  inline bool state() const { return state_; }

 private:
  int pin_num_;
  bool state_;
};

class PwmOutputPin {
 public:
  inline explicit PwmOutputPin(int pin_num, size_t frequency = 50) : pin_num_{pin_num}, frequency_(frequency) {
    StartEmulation();
  }

  ~PwmOutputPin() {
    stop_emulation_ = true;
    if (emulation_thread_.joinable()) {
      emulation_thread_.join();
    }
  }

  inline void Write(double duty) { duty_ = std::clamp(duty, 0.0, 1.0); }

  inline double duty() const { return duty_; }

 private:
  void StartEmulation() {
    emulation_thread_ = std::thread([this]() {
      const auto period = std::chrono::microseconds(static_cast<int>(1e6 / frequency_));
      while (!stop_emulation_) {
        if (duty_ > 0.0) {
          std::printf("Pin %d HIGH\n", pin_num_);
          std::this_thread::sleep_for(period * duty_);
        }
        if (duty_ < 1.0) {
          std::printf("Pin %d LOW\n", pin_num_);
          std::this_thread::sleep_for(period * (1.0 - duty_));
        }
      }
    });
  }

  std::thread emulation_thread_;
  const int pin_num_;
  const size_t frequency_;
  double duty_{0.0};
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

#endif
