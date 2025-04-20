
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

}  // namespace rm

#endif  // UTILS_HPP