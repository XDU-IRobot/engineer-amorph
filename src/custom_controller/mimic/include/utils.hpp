
#ifndef UTILS_HPP
#define UTILS_HPP

#include "typedefs.hpp"

#include <string>

namespace rm {

constexpr u8 CRC8_INIT = 0xff;
constexpr u16 CRC16_INIT = 0xffff;

u8 Crc8(const u8 *input, usize len, u8 init);
u8 Crc8(const std::string &input, u8 init);

u16 Crc16(const u8 *input, usize len, u16 init);
u16 Crc16(const std::string &input, u16 init);

/**
 * @brief   把输入值限制在一个规定的周期内
 * @note    例如输入值为370，周期为360，则输出是10
 * @param   input       输入值
 * @param   min_value   周期开始
 * @param   max_value   周期结束
 * @return              限制到一个周期内的值
 */
inline f32 LoopConstrain(f32 input, f32 min_value, f32 max_value) {
  f32 cycle = max_value - min_value;
  if (cycle < 0) {
    return input;
  }

  if (input > max_value) {
    while (input > max_value) {
      input -= cycle;
    }
  } else if (input < min_value) {
    while (input < min_value) {
      input += cycle;
    }
  }
  return input;
}

}  // namespace rm

#endif  // UTILS_HPP