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

struct __attribute__((packed)) SystemStatus {
  bool left_j1 : 1;
  bool left_j2 : 1;
  bool left_j3 : 1;
  bool left_j4 : 1;
  bool left_j5 : 1;
  bool left_j6 : 1;
  bool left_gripper : 1;
  bool right_j1 : 1;
  bool right_j2 : 1;
  bool right_j3 : 1;
  bool right_j4 : 1;
  bool right_j5 : 1;
  bool right_j6 : 1;
  bool right_gripper : 1;
  bool front_steer : 1;
  bool front_wheel : 1;
  bool left_steer : 1;
  bool left_wheel : 1;
  bool right_steer : 1;
  bool right_wheel : 1;
  bool gimbal_yaw : 1;
  bool camera_lift : 1;
  uint8_t left_j3_temp;
  uint8_t left_gripper_temp;
  uint8_t right_j3_temp;
  uint8_t right_gripper_temp;
  uint8_t fsm_state;
};

struct __attribute__((packed)) EnergyUnitDetection {
  bool detected;
  float p_xyz[3];
  float q_xyzw[4];
};

/// @brief STM32反馈给客户端的数据。车->客户端一次可以发300字节
struct __attribute__((packed)) RobotFeedBack {
  JointState js;
  SystemStatus ss;
  EnergyUnitDetection eud[8];
};
static_assert(sizeof(RobotFeedBack) <= 300);

//********************************************* */
//********************************************* */
//********************************************* */

/// @brief 下面是自定义客户端通过CustomControl帧发给机器人的帧，CmdId放在第一个字节，对应不同的命令格式
enum CmdId {
  kJoy = 0,
};

struct __attribute__((packed)) Joy {
  const uint8_t cmd_id = kJoy;

  bool connected : 1;
  bool btn_a : 1;
  bool btn_b : 1;
  bool btn_x : 1;
  bool btn_y : 1;
  bool btn_lb : 1;
  bool btn_rb : 1;
  bool btn_view : 1;
  bool btn_menu : 1;
  bool btn_share : 1;
  bool btn_xbox : 1;
  bool btn_ls : 1;
  bool btn_rs : 1;
  bool dpad_up : 1;
  bool dpad_down : 1;
  bool dpad_left : 1;
  bool dpad_right : 1;
  float left_x;
  float left_y;
  float right_x;
  float right_y;
  float lt;
  float rt;
};
static_assert(sizeof(Joy) <= 30);

}  // namespace schema
}  // namespace amorph