#pragma once

#include <cstdint>

#include <librm.hpp>

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
  uint8_t vehicle_config;
  uint8_t left_arm_fsm_state;
  uint8_t right_arm_fsm_state;
};

struct __attribute__((packed)) EnergyUnitDetection {
  bool detected;
  float p_xyz[3];
  float q_xyzw[4];
};

/// @brief STM32反馈给客户端的数据。车->客户端 300B/50hz
struct __attribute__((packed)) RobotFeedBack {
  JointState js;
  SystemStatus ss;
  EnergyUnitDetection eud[8];
};
static_assert(sizeof(RobotFeedBack) <= 300);

//********************************************* */
//********************************************* */
//********************************************* */

/// @note 客户端->车 30B/75hz

/// @brief 下面是自定义客户端通过CustomControl帧发给机器人的帧，CmdId放在第一个字节，对应不同的命令格式
enum CmdId {
  kJoy = 0,
  kArmControl,
};

struct __attribute__((packed)) JoyCompressed {
  uint8_t cmd_id = kJoy;

  bool joystick_connected : 1;
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
  rm::modules::f16 left_x;
  rm::modules::f16 left_y;
  rm::modules::f16 right_x;
  rm::modules::f16 right_y;
  rm::modules::f16 lt;
  rm::modules::f16 rt;

  bool imu_connected : 1;
  float imu_roll;
  float imu_pitch;
  float imu_yaw;
};
static_assert(sizeof(JoyCompressed) <= 30);

struct Joy {
  uint8_t cmd_id = kJoy;

  bool joystick_connected : 1;
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

  bool imu_connected : 1;
  float imu_roll;
  float imu_pitch;
  float imu_yaw;

  Joy() = default;
  explicit Joy(const JoyCompressed &compressed) : Joy(FromCompressed(compressed)) {}

  JoyCompressed ToCompressed() const {
    JoyCompressed compressed{};
    compressed.joystick_connected = joystick_connected;
    compressed.btn_a = btn_a;
    compressed.btn_b = btn_b;
    compressed.btn_x = btn_x;
    compressed.btn_y = btn_y;
    compressed.btn_lb = btn_lb;
    compressed.btn_rb = btn_rb;
    compressed.btn_view = btn_view;
    compressed.btn_menu = btn_menu;
    compressed.btn_share = btn_share;
    compressed.btn_xbox = btn_xbox;
    compressed.btn_ls = btn_ls;
    compressed.btn_rs = btn_rs;
    compressed.dpad_up = dpad_up;
    compressed.dpad_down = dpad_down;
    compressed.dpad_left = dpad_left;
    compressed.dpad_right = dpad_right;
    compressed.left_x = rm::modules::F32ToF16(left_x);
    compressed.left_y = rm::modules::F32ToF16(left_y);
    compressed.right_x = rm::modules::F32ToF16(right_x);
    compressed.right_y = rm::modules::F32ToF16(right_y);
    compressed.lt = rm::modules::F32ToF16(lt);
    compressed.rt = rm::modules::F32ToF16(rt);

    compressed.imu_connected = imu_connected;
    compressed.imu_roll = imu_roll;
    compressed.imu_pitch = imu_pitch;
    compressed.imu_yaw = imu_yaw;

    return compressed;
  }

  static Joy FromCompressed(const JoyCompressed &compressed) {
    Joy joy{};
    joy.joystick_connected = compressed.joystick_connected;
    joy.btn_a = compressed.btn_a;
    joy.btn_b = compressed.btn_b;
    joy.btn_x = compressed.btn_x;
    joy.btn_y = compressed.btn_y;
    joy.btn_lb = compressed.btn_lb;
    joy.btn_rb = compressed.btn_rb;
    joy.btn_view = compressed.btn_view;
    joy.btn_menu = compressed.btn_menu;
    joy.btn_share = compressed.btn_share;
    joy.btn_xbox = compressed.btn_xbox;
    joy.btn_ls = compressed.btn_ls;
    joy.btn_rs = compressed.btn_rs;
    joy.dpad_up = compressed.dpad_up;
    joy.dpad_down = compressed.dpad_down;
    joy.dpad_left = compressed.dpad_left;
    joy.dpad_right = compressed.dpad_right;
    joy.left_x = rm::modules::F16ToF32(compressed.left_x);
    joy.left_y = rm::modules::F16ToF32(compressed.left_y);
    joy.right_x = rm::modules::F16ToF32(compressed.right_x);
    joy.right_y = rm::modules::F16ToF32(compressed.right_y);
    joy.lt = rm::modules::F16ToF32(compressed.lt);
    joy.rt = rm::modules::F16ToF32(compressed.rt);

    joy.imu_connected = compressed.imu_connected;
    joy.imu_roll = compressed.imu_roll;
    joy.imu_pitch = compressed.imu_pitch;
    joy.imu_yaw = compressed.imu_yaw;

    return joy;
  }
};

struct __attribute__((packed)) ArmCommandCompressed {
  const uint8_t cmd_id = kArmControl;

  bool take_control_left : 1;
  bool take_control_right : 1;
  bool gripper_open_left : 1;
  bool gripper_open_right : 1;
  rm::modules::f16 left_joint_pos[6];
  rm::modules::f16 right_joint_pos[6];
};

struct ArmCommand {
  bool take_control_left : 1;
  bool take_control_right : 1;
  bool gripper_open_left : 1;
  bool gripper_open_right : 1;
  float left_joint_pos[6];
  float right_joint_pos[6];

  ArmCommand() = default;

  explicit ArmCommand(ArmCommandCompressed compressed) {
    take_control_left = compressed.take_control_left;
    take_control_right = compressed.take_control_right;
    gripper_open_left = compressed.gripper_open_left;
    gripper_open_right = compressed.gripper_open_right;
    for (size_t i = 0; i < 6; ++i) {
      left_joint_pos[i] = rm::modules::F16ToF32(compressed.left_joint_pos[i]);
      right_joint_pos[i] = rm::modules::F16ToF32(compressed.right_joint_pos[i]);
    }
  }

  ArmCommandCompressed ToCompressed() const {
    ArmCommandCompressed compressed{};
    compressed.take_control_left = take_control_left;
    compressed.take_control_right = take_control_right;
    compressed.gripper_open_left = gripper_open_left;
    compressed.gripper_open_right = gripper_open_right;
    for (size_t i = 0; i < 6; ++i) {
      compressed.left_joint_pos[i] = rm::modules::F32ToF16(left_joint_pos[i]);
      compressed.right_joint_pos[i] = rm::modules::F32ToF16(right_joint_pos[i]);
    }
    return compressed;
  }
};

struct __attribute__((packed)) TwoPointRange {
  bool range_l_valid : 1;
  bool range_r_valid : 1;
  float range_l;
  float range_r;
};

}  // namespace schema
}  // namespace amorph