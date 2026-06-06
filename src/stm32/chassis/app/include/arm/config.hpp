#pragma once

#include <librm.hpp>

#include "et16s.hpp"
#include "dynamics.hpp"
#include "m_pi_f.hpp"
#include "shared_resources.hpp"
#include "etl/message_types.h"

using namespace rm::modules::angle_literals;

struct ArmConfig {
  struct PDParams {
    float kp;
    float kd;
  };
  enum ArmId {
    kLeft,
    kRight,
  } arm_id;

  rm::hal::CanInterface *can;
  size_t host_comm_id;  ///< 此臂在通信总线上发送反馈帧的 CAN ID

  // 电机配置
  uint8_t gripper_motor_id;                                  ///< 夹爪电机 ID（LkMotorMS）
  uint8_t j6_motor_id;                                       ///< j6电机ID（GM6020）
  rm::device::DmMotorMit::Settings dm_settings[6];           ///< j1~j5+夹爪 达妙电机参数
  rm::modules::MotorCalibrator::MotorConfig calibrators[7];  ///< 关节零点、转向校准配置

  // 控制参数
  PDParams pd_params[7];  ///< j1~j6+夹爪 PD 增益

  // 运动学 / 动力学
  std::array<std::pair<float, float>, 6> joint_limits;  ///< {min, max}（逻辑坐标，rad）
  ArmDynamics::Params dynamics_params;                  ///< 动力学参数

  std::array<rm::modules::TrajectoryLimiter, 6> trajectory_limiters;

  // 遥控器通道映射
  int rc_ch_manual_gripper_switch;  ///< Manual模式下开关夹爪的通道
  int rc_ch_mode_switch;            ///< 模式切换拨杆通道
  int rc_ch_next_joint{rc_ch::SH};  ///< 切换关节拨杆通道

  etl::message_router_id_t fsm_msg_router_id;

  static ArmConfig MakeLeft() {
    using namespace rm::modules::angle_literals;
    return {
        .arm_id = kLeft,
        .can = &SharedResources::GetInstance().left_can,
        .host_comm_id = 0x242,
        .gripper_motor_id = 2,
        .j6_motor_id = 2,
        .dm_settings =
            {//
             // j1
             {.master_id = 0xb1,
              .slave_id = 0xa1,
              .p_max = 3.141593f,
              .v_max = 20.f,
              .t_max = 28.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}},
             // j2
             {.master_id = 0xb2,
              .slave_id = 0xa2,
              .p_max = 3.141593f,
              .v_max = 45.f,
              .t_max = 54.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}},
             // j3
             {.master_id = 0xb3,
              .slave_id = 0xa3,
              .p_max = 12.5f,
              .v_max = 20.f,
              .t_max = 28.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}},
             // j4
             {.master_id = 0xb4,
              .slave_id = 0xa4,
              .p_max = 12.5f,
              .v_max = 30.f,
              .t_max = 10.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}},
             // j5
             {.master_id = 0xb5,
              .slave_id = 0xa5,
              .p_max = 3.141593f,
              .v_max = 30.f,
              .t_max = 10.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}},
             // gripper
             {.master_id = 0xb6,
              .slave_id = 0xa6,
              .p_max = 12.5f,
              .v_max = 30.f,
              .t_max = 10.f,
              .kp_range = {0, 500},
              .kd_range = {0, 50}}},
        .calibrators =
            {
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = -(M_PIf - std::atan2f(84.5f, 415.f))},
                {.reverse = false, .zero_offset = -(std::atan2f(84.5f, 415.f) - (90_deg).rad())},
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 1894.f / 8191.f * 2.f * M_PIf},
                {.reverse = false, .zero_offset = 0.f},
            },
        .pd_params =
            {
                {.kp = 150.f, .kd = 30.f},
                {.kp = 140.f, .kd = 40.f},
                {.kp = 120.f, .kd = 40.f},
                {.kp = 35.f, .kd = 10.f},
                {.kp = 35.f, .kd = 8.f},
                {.kp = 30.f, .kd = 10.f},
                {.kp = 0.f, .kd = 1.f},
                //
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
                // {.kp = 0.f, .kd = 0.f},
            },
        .joint_limits = {{
            {-(45_deg).rad(), (30_deg).rad()},    // j1 yaw
            {-M_PIf * 2.f, M_PIf * 2.f},          // j2 pitch1
            {-M_PIf * 2.f, M_PIf * 2.f},          // j3 pitch2
            {-M_PIf, M_PIf},                      // j4 roll1
            {-(100_deg).rad(), (100_deg).rad()},  // j5 pitch3
            {-M_PIf * 2.f, M_PIf * 2.f},          // j6, roll2, continuous joint (+-2π soft limit)
        }},
        .dynamics_params = {.b1 = 1.733725f, .b2 = 1.1055f},
        .trajectory_limiters = {{{3.775f, 5.75f},  //
                                 {2.9f, 6.53f},    //
                                 {3.575f, 3.f},    //
                                 {10.f, 50.f},     //
                                 {10.f, 50.f},     //
                                 {17.f, 50.f}}},
        .rc_ch_manual_gripper_switch = rc_ch::LS,
        .rc_ch_mode_switch = rc_ch::SB,
        .fsm_msg_router_id = 1,
    };
  }
  static ArmConfig MakeRight() {
    using namespace rm::modules::angle_literals;
    return {
        .arm_id = kRight,
        .can = &SharedResources::GetInstance().right_can,
        .host_comm_id = 0x232,
        .gripper_motor_id = 3,
        .j6_motor_id = 1,
        .dm_settings = {{.master_id = 0xb1,
                         .slave_id = 0xa1,
                         .p_max = 3.141593f,
                         .v_max = 20.f,
                         .t_max = 28.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}},
                        {.master_id = 0xb2,
                         .slave_id = 0xa2,
                         .p_max = 3.141593f,
                         .v_max = 45.f,
                         .t_max = 54.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}},
                        {.master_id = 0xb3,
                         .slave_id = 0xa3,
                         .p_max = 3.141593f,
                         .v_max = 20.f,
                         .t_max = 28.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}},
                        {.master_id = 0xb4,
                         .slave_id = 0xa4,
                         .p_max = 3.141593f,
                         .v_max = 30.f,
                         .t_max = 10.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}},
                        {.master_id = 0xb5,
                         .slave_id = 0xa5,
                         .p_max = 3.141593f,
                         .v_max = 30.f,
                         .t_max = 10.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}},
                        // gripper
                        {.master_id = 0xb6,
                         .slave_id = 0xa6,
                         .p_max = 10000.f,
                         .v_max = 30.f,
                         .t_max = 10.f,
                         .kp_range = {0, 500},
                         .kd_range = {0, 50}}},
        .calibrators =
            {
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = true, .zero_offset = M_PIf - std::atan2f(84.5f, 415.f)},
                {.reverse = true, .zero_offset = std::atan2f(84.5f, 415.f) - (90_deg).rad()},
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 611.f / 8191.f * 2.f * M_PIf},
                {.reverse = false, .zero_offset = 0.f},
            },
        .pd_params =
            {
                {.kp = 150.f, .kd = 30.f},
                {.kp = 140.f, .kd = 40.f},
                {.kp = 120.f, .kd = 40.f},
                {.kp = 35.f, .kd = 10.f},
                {.kp = 35.f, .kd = 8.f},
                {.kp = 30.f, .kd = 10.f},
                {.kp = 0.f, .kd = 2.f},
            },
        .joint_limits = {{
            {-(30_deg).rad(), (45_deg).rad()},    // j1 yaw
            {-M_PIf * 2.f, M_PIf * 2.f},          // j2 pitch1
            {-M_PIf * 2.f, M_PIf * 2.f},          // j3 pitch2
            {-M_PIf, M_PIf},                      // j4 roll1
            {-(100_deg).rad(), (100_deg).rad()},  // j5 pitch3
            {-M_PIf * 2.f, M_PIf * 2.f},          // j6, roll2, continuous joint (+-2π soft limit)
        }},
        .dynamics_params = {.b1 = 1.733725f, .b2 = 1.1055f},
        .trajectory_limiters = {{{3.775f, 5.75f},  //
                                 {3.575f, 10.5f},    //
                                 {3.575f, 3.5f},    //
                                 {10.f, 50.f},     //
                                 {10.f, 50.f},     //
                                 {17.f, 50.f}}},
        .rc_ch_manual_gripper_switch = rc_ch::RS,
        .rc_ch_mode_switch = rc_ch::SC,
        .fsm_msg_router_id = 2,
    };
  }
};