
#ifndef ARM_CUSTOM_CONTROLLER_HPP
#define ARM_CUSTOM_CONTROLLER_HPP

namespace fluorite::custom_controller {

struct __attribute__((packed)) CartesianCommand {
  float pos_x;
  float pos_y;
  float pos_z;
  float ori_w;
  float ori_x;
  float ori_y;
  float ori_z;
};

struct __attribute__((packed)) JointCommand {
  bool ok;
  float joint_positions[6];
};

}  // namespace fluorite::custom_controller

#endif