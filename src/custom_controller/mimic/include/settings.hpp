
#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <array>

namespace settings {

inline constexpr std::array<float, 6> kEncoderOffsets{
    2.61, 0.94, 0.54, 3.46, 1.33, 0.06,
};
inline constexpr std::array<bool, 6> kEncoderReverse{
    false, false, false, false, false, false,
};

}  // namespace settings

#endif  // SETTINGS_HPP
