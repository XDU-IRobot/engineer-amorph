#pragma once

#include <array>
#include <cmath>
#include <optional>

class CartesianLimit {
 public:
  CartesianLimit(float l1, float l2, float max_x) : l1_(l1), l2_(l2), max_x_(max_x) {}

  /// @brief  对给定的关节角 (q1, q2) 施加 x < max_x_ 限位。
  ///         若 FK 末端 x > max_x_，则沿 y 方向垂直投影至 (max_x_, y)，
  ///         并对 clamped 位置求逆运动学 (elbow-up)。
  /// @return 限位后的 (q1, q2)；若位置不可达则返回 nullopt。
  std::optional<std::array<float, 2>> Limit(float q1, float q2) const {
    // fk
    float c1 = std::cos(q1);
    float s1 = std::sin(q1);
    float c12 = std::cos(q1 + q2);
    float s12 = std::sin(q1 + q2);

    float x = l1_ * c1 + l2_ * c12;
    float y = l1_ * s1 + l2_ * s12;

    // 未超限则原样返回
    if (x <= max_x_) {
      return std::array<float, 2>{q1, q2};
    }

    // 限位则垂直投影至x = max_x_
    float xc = max_x_;
    float yc = y;

    // 工作空间检查
    float r2 = xc * xc + yc * yc;
    float r_min = std::abs(l1_ - l2_);
    float r_max = l1_ + l2_;
    if (r2 < r_min * r_min || r2 > r_max * r_max) {
      return std::nullopt;
    }

    // IK, elbow-up
    float c2 = (r2 - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_);
    if (c2 < -1.0f || c2 > 1.0f) {
      return std::nullopt;
    }

    float q2_clamped = -std::acos(c2);  // elbow-up: θ₂ < 0
    float q1_clamped = std::atan2(yc, xc) - std::atan2(l2_ * std::sin(q2_clamped), l1_ + l2_ * std::cos(q2_clamped));

    return std::array<float, 2>{q1_clamped, q2_clamped};
  }

 private:
  float l1_;
  float l2_;
  float max_x_;
};