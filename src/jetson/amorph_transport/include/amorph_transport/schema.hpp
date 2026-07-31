#pragma once

#include <cstdint>

namespace amorph {
namespace schema {

struct __attribute__((packed)) JointState {
  float left[6];
  float right[6];
  bool left_gripper_open : 1;
  bool right_gripper_open : 1;
};

}  // namespace schema
}  // namespace amorph