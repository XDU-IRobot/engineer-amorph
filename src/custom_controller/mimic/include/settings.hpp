
#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <array>
#include <utility>

namespace settings {

inline constexpr std::array<size_t, 6> kJointMapping{4, 0, 1, 2, 3, 5};
inline constexpr std::array<float, 6> kEncoderOffsets{
    1.31, 5.34, 5.68, 3.48, 4.78, 0.0,
};
inline constexpr std::array<std::pair<float, float>, 6> kJointLimits{
    std::make_pair(-2.355, 2.355),    //
    std::make_pair(0.0, 2.7911),      //
    std::make_pair(0.0, 2.7911),      //
    std::make_pair(-2.6167, 2.6167),  //
    std::make_pair(-1.57, 1.57),      //
    std::make_pair(-3.14, 3.14),      //
};
inline constexpr std::array<bool, 6> kEncoderReverse{
    false, true, true, false, true, false,
};

}  // namespace settings

#endif  // SETTINGS_HPP
