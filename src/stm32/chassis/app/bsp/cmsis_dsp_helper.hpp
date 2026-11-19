#pragma once

#include <arm_math.h>

namespace details {
inline float __attribute__((always_inline)) ArmSqrt(float in) {
  float out;
  arm_sqrt_f32(in, &out);
  return out;
}
inline float __attribute__((always_inline)) ArmAtan2(float y, float x) {
  float out;
  arm_atan2_f32(y, x, &out);
  return out;
}
}  // namespace details

#define COS_FN(x) arm_cos_f32(x)
#define SIN_FN(x) arm_sin_f32(x)
#define ABS_FN(x) std::abs(x)
#define ATAN2_FN(y, x) details::ArmAtan2(y, x)
#define SQRT_FN(x) details::ArmSqrt(x)

// #define COS_FN(x) std::cos(x)
// #define SIN_FN(x) std::sin(x)