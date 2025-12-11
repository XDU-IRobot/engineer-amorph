
#pragma once

/**
 * @brief 编码器计圈器，用于跟踪没有绝对位置的编码器的过圈数
 */
class EncoderCounter {
 public:
  EncoderCounter() = default;
  explicit EncoderCounter(float cycle) : cycle_(cycle) {}

  void SetCycle(float cycle) { cycle_ = cycle; }

  /**
   * @brief 重置计圈器状态
   * @param cycles        重置的圈数
   * @param value         重置的编码器读数
   */
  void Reset(int cycles = 0, float value = 0) {
    revolutions_ = cycles;
    last_ecd_ = value;
    initialized_ = false;  // 下次 Update 会用传入的 ecd 初始化
  }

  /**
   * @brief 更新计圈器状态
   * @param ecd          当前编码器读数
   */
  void Update(float ecd) {
    if (!initialized_) {
      last_ecd_ = ecd;
      initialized_ = true;
      return;
    }

    // 计算这次相较于上次的增量
    const float delta = ecd - last_ecd_;

    if (delta > 0) {
      if (delta > cycle_) {
        // 反向跨圈：圈数减1
        --revolutions_;
        last_ecd_ = ecd;
      } else {
        // 正常小幅正转
        last_ecd_ = ecd;
      }
    } else if (delta < 0) {
      if (delta < -cycle_) {
        // 正向跨圈：圈数加1
        ++revolutions_;
        last_ecd_ = ecd;
      } else {
        // 正常小幅反转
        last_ecd_ = ecd;
      }
    }
  }

  int revolutions() const { return revolutions_; }
  float last_ecd() const { return last_ecd_; }
  float abs_position() const { return revolutions_ * cycle_ + last_ecd_; }

 private:
  float cycle_{2 * M_PI};
  int revolutions_{0};  ///< 圈数
  float last_ecd_{0};
  bool initialized_{false};
};