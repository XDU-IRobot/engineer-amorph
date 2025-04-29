#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <cmath>

/**
 * @brief quarticInOut的反函数
 */
template <typename T>
T QuarticInOutEasingInversed(float percentage, T start, T end) {
  const T delta = end - start;
  if (percentage < 0.5) {
    return static_cast<T>(start + delta * std::pow(2 * percentage, 0.25) / 2);
  }
  return static_cast<T>(end - delta * std::pow(2 * (1 - percentage), 0.25) / 2);
}

/**
 * @brief sineInOut的反函数
 */
template <typename T>
T SineInOutEasingInversed(float percentage, T start, T end) {
  const T delta = end - start;
  return static_cast<T>(std::abs(start + delta * (std::asin(2 * percentage - 1) / M_PI + 0.5)));
}

#endif