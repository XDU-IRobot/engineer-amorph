#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 有限傅里叶级数轨迹发生器
 *
 * 轨迹方程为（振荡部分）：
 * q_osc(t) = sum_{k=1}^N [ (a_k / (k*wf)) * sin(k*wf*t) - (b_k / (k*wf)) * cos(k*wf*t) ]
 * dq_osc(t) = sum_{k=1}^N [ a_k * cos(k*wf*t) + b_k * sin(k*wf*t) ]
 * ddq_osc(t) = sum_{k=1}^N [ -a_k * k*wf * sin(k*wf*t) + b_k * k*wf * cos(k*wf*t) ]
 *
 * 输出映射到用户指定的 [q_min, q_max] 区间：
 *   q(t)   = center + scale * q_osc(t)
 *   dq(t)  = scale * dq_osc(t)
 *   ddq(t) = scale * ddq_osc(t)
 *
 * 其中 scale 基于各谐波振幅之和的保守上界计算，确保 q(t) 不超出 [q_min, q_max]。
 *
 * @tparam N 傅里叶谐波项数（通常选 5 ）
 */
template <size_t N>
class FourierTrajectoryGenerator {
  float wf_;        // 基频 (rad/s)
  float q_center_;  // 输出中心 = (q_min + q_max) / 2
  float q_scale_;   // 输出缩放因子 = half_range / amp_bound
  float a_[N];      // 傅里叶系数 a
  float b_[N];      // 傅里叶系数 b

 public:
  /**
   * @brief 构造函数
   * @param base_freq_hz 基频(Hz)，例如 0.1Hz 即周期10s
   * @param q_min 输出位置下限 (rad)
   * @param q_max 输出位置上限 (rad)
   * @param a_coeffs 长度为 N 的系数组 a
   * @param b_coeffs 长度为 N 的系数组 b
   */
  FourierTrajectoryGenerator(float base_freq_hz, float q_min, float q_max, const std::array<float, N> a_coeffs,
                             const std::array<float, N> b_coeffs) {
    wf_ = 2.0f * M_PI * base_freq_hz;
    q_center_ = 0.5f * (q_min + q_max);

    // 保守振幅上界：各谐波振幅之和
    float amp_bound = 0.0f;
    for (size_t i = 0; i < N; ++i) {
      a_[i] = a_coeffs[i];
      b_[i] = b_coeffs[i];
      float k = static_cast<float>(i + 1);
      amp_bound += std::sqrt(a_[i] * a_[i] + b_[i] * b_[i]) / (k * wf_);
    }

    q_scale_ = (amp_bound > 0.0f) ? (0.5f * (q_max - q_min)) / amp_bound : 1.0f;
  }

  struct Output {
    float q;
    float dq;
    float ddq;
  };

  /**
   * @brief 计算给定时间 t 的理想位置、速度、加速度
   * @param t 当前时间(秒)
   * @return Output 包含目标位置 q (rad)、速度 dq (rad/s)、加速度 ddq (rad/s²)
   */
  Output evaluate(float t) const {
    Output output;
    output.q = 0.0f;
    output.dq = 0.0f;
    output.ddq = 0.0f;

    for (size_t i = 0; i < N; ++i) {
      float k = static_cast<float>(i + 1);
      float kwf = k * wf_;
      float phase = kwf * t;

      // 注意: 在极端性能敏感时，你可以将这里的 std::sin/cos 替换为 arm_sin_f32/arm_cos_f32
      float s = std::sin(phase);
      float c = std::cos(phase);

      // 叠加振荡部分
      output.q += (a_[i] / kwf) * s - (b_[i] / kwf) * c;
      output.dq += a_[i] * c + b_[i] * s;
      output.ddq += -a_[i] * kwf * s + b_[i] * kwf * c;
    }
    // 映射到 [q_min, q_max]
    output.q = q_center_ + q_scale_ * output.q;
    output.dq *= q_scale_;
    output.ddq *= q_scale_;
    return output;
  }
};