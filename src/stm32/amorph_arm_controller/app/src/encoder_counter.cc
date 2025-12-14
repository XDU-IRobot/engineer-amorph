
#include "encoder_counter.hpp"

#include <librm.hpp>

using rm::modules::Wrap;

EncoderCounter::EncoderCounter(float gear_ratio, float encoder_cycle)
    : gear_ratio_(gear_ratio),
      encoder_cycle_(encoder_cycle),
      last_encoder_(0.0f),
      rotation_count_(0),
      first_update_(true) {}

/**
 * @return  输出轴绝对位置，单位：rad
 */
float EncoderCounter::RealPosition() const {
  // 电机总位置 = (圈数 * 编码器周期 + 当前编码器读数)
  // 关节位置 = 电机位置 / 减速比
  float motor_position = rotation_count_ * encoder_cycle_ + last_encoder_;
  return motor_position / gear_ratio_;
}

/**
 * @brief 更新编码器累加值
 * @param encoder 编码器读数
 */
void EncoderCounter::Update(float encoder) {
  // 第一次更新时，直接记录编码器值，不计算圈数变化
  if (first_update_) {
    last_encoder_ = encoder;
    first_update_ = false;
    return;
  }

  // 计算编码器变化量
  float delta = encoder - last_encoder_;

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

/**
 * @brief 重置编码器累加值
 */
void EncoderCounter::Reset() {
  last_encoder_ = 0.0f;
  rotation_count_ = 0;
  first_update_ = true;
}
