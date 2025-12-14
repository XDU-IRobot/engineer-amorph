
#pragma once

/**
 * @brief 计算编码器只有一圈的带减速比的电机驱动的关节的实际位置
 */
class EncoderCounter {
 public:
  EncoderCounter(float gear_ratio,    ///< 减速比
                 float encoder_cycle  ///< 编码器单圈周期
  );
  float RealPosition() const;
  void Update(float encoder);
  void Reset();

 private:
  float gear_ratio_;     ///< 减速比
  float encoder_cycle_;  ///< 编码器单圈周期
  float last_encoder_;   ///< 上一次编码器读数
  int rotation_count_;   ///< 圈数计数
  bool first_update_;    ///< 是否第一次更新
};
