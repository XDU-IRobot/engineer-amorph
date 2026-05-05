#pragma once

#include <librm.hpp>

#include "et16s.hpp"
#include "dynamics.hpp"
#include "m_pi_f.hpp"
#include "shared_resources.hpp"

namespace detail {
struct PDParams {
  float kp;
  float kd;
};
}  // namespace detail

using namespace rm::modules::angle_literals;

struct ArmConfig {
  rm::hal::CanInterface *can;
  size_t host_comm_id;  ///< 此臂在通信总线上发送反馈帧的 CAN ID

  // 电机配置
  uint8_t gripper_motor_id;                                  ///< 夹爪电机 ID（LkMotorMS）
  uint8_t j6_motor_id;                                       ///< j6电机ID（GM6020）
  rm::device::DmMotorMit::Settings dm_settings[5];           ///< j1~j5 达妙电机参数
  rm::modules::MotorCalibrator::MotorConfig calibrators[7];  ///< 关节零点、转向校准配置

  // 控制参数
  detail::PDParams pd_params[6];  ///< j1~j6 PD 增益

  // 运动学 / 动力学
  std::array<std::pair<float, float>, 7> joint_limits{{
      {-(10_deg).rad(), (10_deg).rad()},    // j1 yaw
      {-M_PIf, M_PIf},                      // j2 pitch1
      {-M_PIf, M_PIf},                      // j3 pitch2
      {-M_PIf, M_PIf},                      // j4 roll1
      {-(100_deg).rad(), (100_deg).rad()},  // j5 pitch3
      {-M_PIf, M_PIf},                      // dummy, j6, roll2, continuous joint
      {-M_PIf, M_PIf},                      // dummy, gripper, continuous joint
  }};  ///< {min, max}（逻辑坐标，rad），两边的臂是一样的
  ArmDynamics::Params dynamics_params;  ///< 动力学参数

  std::array<rm::modules::TrajectoryLimiter, 7> trajectory_limiters;

  // 遥控器通道映射
  int rc_ch_mode_switch;            ///< 模式切换拨杆通道
  int rc_ch_next_joint{rc_ch::SH};  ///< 切换关节拨杆通道

  static ArmConfig MakeRight() {
    return {
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
                         .kd_range = {0, 50}}},
        .calibrators =
            {
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = true, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
                {.reverse = false, .zero_offset = 0.f},
            },
        .pd_params =
            {
                {60.f, 25.f},
                {140.f, 40.f},
                {120.f, 40.f},
                {35.f, 10.f},
                {35.f, 8.f},
                {30.f, 10.f},
            },
        .dynamics_params = {1.2725f, 0.7917f},
        .trajectory_limiters = {{{2.175f, 3.75f},
                                 {2.175f, 2.5f},
                                 {2.175f, 2.5f},
                                 {3.1f, 15.f},
                                 {2.61f, 3.75f},
                                 {3.1f, 15.f},
                                 {2.61f, 5.f}}},
        .rc_ch_mode_switch = rc_ch::SC,
    };
  }
};