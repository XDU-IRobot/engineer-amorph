
#include <librm.hpp>

namespace schema {
struct __attribute__((packed)) EulerAngle {
  constexpr static uint8_t kFrameId = 0xa1;
  rm::modules::f16 pitch;
  rm::modules::f16 roll;
  rm::modules::f16 yaw;
};

static_assert(sizeof(EulerAngle) <= 8);
}  // namespace schema