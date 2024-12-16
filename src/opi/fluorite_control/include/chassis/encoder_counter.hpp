
#ifndef CHASSIS_ENCODER_COUNTER_HPP
#define CHASSIS_ENCODER_COUNTER_HPP

/**
 * @file    encoder_counter.hpp
 * @brief   用软件方式实现单圈编码器的绝对值计数
 */

template <typename T>
class EncoderCounter {
 public:
  EncoderCounter() = delete;
  ~EncoderCounter() = default;
  explicit EncoderCounter(T cycle) : cycle_{cycle} {}

  void Update(T encoder) {
    if (encoder < previous_ && previous_ - encoder > cycle_ / 2) {
      cycles_++;
    } else if (encoder > previous_ && encoder - previous_ > cycle_ / 2) {
      cycles_--;
    }
    previous_ = encoder;
  }

  void Reset() {
    previous_ = 0;
    cycles_ = 0;
  }

  [[nodiscard]] T value() const { return previous_ + cycles_ * cycle_; }

 private:
  T previous_{};
  int cycles_{};
  const T cycle_{};
};

#endif  // CHASSIS_ENCODER_COUNTER_HPP