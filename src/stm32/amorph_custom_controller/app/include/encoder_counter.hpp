
#pragma once

/**
 * @brief 计算编码器只有一圈的电机驱动的关节的实际位置
 */
class EncoderCounter {
 public:
  /**
   * @param encoder_cycle 编码器走一圈的周期
   */
  explicit EncoderCounter(float encoder_cycle) : encoder_cycle_(encoder_cycle) {}

  float RealPosition() const {
    // 电机总位置 = (圈数 * 编码器周期 + 当前编码器读数)
    return rotation_count_ * encoder_cycle_ + last_encoder_;
  }

  void Update(float encoder) {
    // 第一次更新时，直接记录编码器值，不计算圈数变化
    if (first_update_) {
      last_encoder_ = encoder;
      first_update_ = false;
      return;
    }

    // 计算编码器变化量
    const float delta = encoder - last_encoder_;

    // 检测跨越周期边界
    // 如果变化量超过半个周期，说明发生了跨越
    if (delta > encoder_cycle_ / 2.0f) {
      // 编码器从大值跳到小值，向下跨越（逆时针转动穿过0点）
      rotation_count_--;
    } else if (delta < -encoder_cycle_ / 2.0f) {
      // 编码器从小值跳到大值，向上跨越（顺时针转动穿过0点）
      rotation_count_++;
    }

    // 更新上次编码器读数
    last_encoder_ = encoder;
  }

  void Reset() {
    last_encoder_ = 0.0f;
    rotation_count_ = 0;
    first_update_ = true;
  }

 private:
  float encoder_cycle_;      ///< 编码器单圈周期
  float last_encoder_{0.f};  ///< 上一次编码器读数
  int rotation_count_{0};    ///< 圈数计数
  bool first_update_{true};  ///< 是否第一次更新
};
