#pragma once

#include <array>
#include <cmath>
#include <optional>

class CartesianLimit {
 public:
  CartesianLimit(float l1, float l2) : l1_(l1), l2_(l2) {}

  /// @brief  对给定的关节角 (q2, q3) 施加 x < max_x_ 限位。
  ///         若 FK 末端 x > max_x_，则沿 y 方向垂直投影至 (max_x_, y)，
  ///         并对 clamped 位置求逆运动学 (elbow-up)。
  /// @return 限位后的 (q2, q3)；若位置不可达则返回 nullopt。
  std::optional<std::array<float, 2>> Limit(float q1, float q2, float q3) {
    // fk
    const float c1 = std::cos(q2);
    const float s1 = std::sin(q2);
    const float c12 = std::cos(q2 + q3);
    const float s12 = std::sin(q2 + q3);

    const float x = l1_ * c1 + l2_ * c12;
    const float y = l1_ * s1 + l2_ * s12;

    max_x_ = CalcMaxX(q1);

    // 未超限则原样返回
    if (x <= max_x_) {
      return std::array<float, 2>{q2, q3};
    }

    // 限位则垂直投影至x = max_x_
    const float xc = max_x_;
    const float yc = y;

    // 工作空间检查
    const float r2 = xc * xc + yc * yc;
    const float r_min = std::abs(l1_ - l2_);
    const float r_max = l1_ + l2_;
    if (r2 < r_min * r_min || r2 > r_max * r_max) {
      return std::nullopt;
    }

    // IK, elbow-up
    const float c2 = (r2 - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_);
    if (c2 < -1.0f || c2 > 1.0f) {
      return std::nullopt;
    }

    const float q2_clamped = -std::acos(c2);  // elbow-up: θ₂ < 0
    const float q1_clamped =
        std::atan2(yc, xc) - std::atan2(l2_ * std::sin(q2_clamped), l1_ + l2_ * std::cos(q2_clamped));

    return std::array<float, 2>{q1_clamped, q2_clamped};
  }

 private:
  static float CalcMaxX(float q1) {
    constexpr static float kDimSide = 0.178f - 0.08f;
    constexpr static float kDimFront = 0.5195f - 0.08f;
    constexpr static float kDimBack = 0.3255f - 0.08f;

    const float cos_q1 = std::cos(q1);
    float x_component = 0.f;
    float y_component = 0.f;
    if (cos_q1 < 0.0f) {
      x_component = kDimBack * cos_q1;
    } else {
      x_component = kDimFront * cos_q1;
    }
    y_component = std::sin(q1) * kDimSide;

    return std::sqrtf(std::powf(x_component, 2.f) + std::powf(y_component, 2.f));
  }

  float l1_;
  float l2_;
  float max_x_{};
};