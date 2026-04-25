/**
 * @file  arm_config.cc
 * @brief 左/右臂 ArmConfig 工厂函数实现
 */

#include "arm_config.hpp"

#include <cmath>

// ---- 辅助：把角度（度）转为弧度 ----
static constexpr float DegToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.f; }

// ======================================================
//                     左 臂
// ======================================================
ArmConfig MakeLeftArmConfig(rm::hal::CanInterface& lower_can, rm::hal::CanInterface& upper_can) {
  ArmConfig cfg;
  cfg.lower_motor_can = &lower_can;
  cfg.upper_motor_can = &upper_can;
  cfg.host_comm_id    = 0x20;
  cfg.gripper_motor_id = 1;

  // ---- 达妙电机参数 （j1~j6） ----
  cfg.dm_settings[0] = {.master_id = 0xb1, .slave_id = 0xa1, .p_max = 3.141593f, .v_max = 20.f,  .t_max = 28.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[1] = {.master_id = 0xb2, .slave_id = 0xa2, .p_max = 3.141593f, .v_max = 45.f,  .t_max = 54.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[2] = {.master_id = 0xb3, .slave_id = 0xa3, .p_max = 3.141593f, .v_max = 20.f,  .t_max = 28.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[3] = {.master_id = 0xb4, .slave_id = 0xa4, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[4] = {.master_id = 0xb5, .slave_id = 0xa5, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[5] = {.master_id = 0xb6, .slave_id = 0xa6, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};

  // ---- 校准配置 ----
  cfg.calibrators[0] = {true,  0.f};
  cfg.calibrators[1] = {false, -DegToRad(90.f - 11.93f)};
  cfg.calibrators[2] = {false,  DegToRad(40.f - 11.93f)};
  cfg.calibrators[3] = {false, 0.f};
  cfg.calibrators[4] = {true,  0.f};
  cfg.calibrators[5] = {false, 0.f};
  cfg.calibrators[6] = {false, 0.f};

  // ---- PD 参数 ----
  cfg.pd_params[0] = {60.f,  25.f};
  cfg.pd_params[1] = {120.f, 35.f};
  cfg.pd_params[2] = {100.f, 35.f};
  cfg.pd_params[3] = {35.f,  15.f};
  cfg.pd_params[4] = {35.f,  15.f};
  cfg.pd_params[5] = {30.f,  10.f};

  // ---- 关节限位 ----
  cfg.joint_limits = {{
      {-0.698132f,  0.349066f},
      { 1.362579f,  2.933375f},
      {-2.933375f,  0.0f},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
      {-static_cast<float>(M_PI) / 2.f, static_cast<float>(M_PI) / 2.f},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
  }};

  // ---- 轨迹参数 {最大速度, 最大加速度} ----
  cfg.trajectory_params = {{
      {2.175f, 3.75f},
      {2.175f, 2.5f},
      {2.175f, 2.5f},
      {3.1f,   15.f},
      {2.61f,  3.75f},
      {3.1f,   15.f},
      {2.61f,  5.f},
  }};

  cfg.dynamics_b1 = 12.470911f;
  cfg.dynamics_b2 = 7.758989f;

  cfg.rc_ch_mode_switch = RcReceiver::kSb;
  cfg.rc_ch_next_joint  = RcReceiver::kSh;

  return cfg;
}

// ======================================================
//                     右 臂
// ======================================================
ArmConfig MakeRightArmConfig(rm::hal::CanInterface& lower_can, rm::hal::CanInterface& upper_can) {
  ArmConfig cfg;
  cfg.lower_motor_can = &lower_can;
  cfg.upper_motor_can = &upper_can;
  cfg.host_comm_id    = 0x30;
  cfg.gripper_motor_id = 3;

  // ---- 达妙电机参数 （j1~j6） ----
  cfg.dm_settings[0] = {.master_id = 0xb1, .slave_id = 0xa1, .p_max = 3.141593f, .v_max = 20.f,  .t_max = 28.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[1] = {.master_id = 0xb2, .slave_id = 0xa2, .p_max = 3.141593f, .v_max = 45.f,  .t_max = 54.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[2] = {.master_id = 0xb3, .slave_id = 0xa3, .p_max = 3.141593f, .v_max = 20.f,  .t_max = 28.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[3] = {.master_id = 0xb4, .slave_id = 0xa4, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[4] = {.master_id = 0xb5, .slave_id = 0xa5, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};
  cfg.dm_settings[5] = {.master_id = 0xb6, .slave_id = 0xa6, .p_max = 3.141593f, .v_max = 30.f,  .t_max = 10.f, .kp_range = {0, 500}, .kd_range = {0, 50}};

  // ---- 校准配置 ----
  cfg.calibrators[0] = {true,  0.f};
  cfg.calibrators[1] = {true,   DegToRad(90.f - 11.93f)};
  cfg.calibrators[2] = {true,  -DegToRad(40.f - 11.93f)};
  cfg.calibrators[3] = {false, 0.f};
  cfg.calibrators[4] = {false, 0.f};
  cfg.calibrators[5] = {false, 0.f};
  cfg.calibrators[6] = {false, 0.f};

  // ---- PD 参数 ----
  cfg.pd_params[0] = {60.f,  25.f};
  cfg.pd_params[1] = {140.f, 40.f};
  cfg.pd_params[2] = {120.f, 40.f};
  cfg.pd_params[3] = {35.f,  15.f};
  cfg.pd_params[4] = {35.f,  10.f};
  cfg.pd_params[5] = {30.f,  10.f};

  // ---- 关节限位 ----
  cfg.joint_limits = {{
      {-0.698132f,  0.349066f},
      { 1.362579f,  2.933375f},
      {-2.933375f, -0.48f},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
      {-static_cast<float>(M_PI) / 2.f, static_cast<float>(M_PI) / 2.f},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
      {-static_cast<float>(M_PI), static_cast<float>(M_PI)},
  }};

  // ---- 轨迹参数 ----
  cfg.trajectory_params = {{
      {2.175f, 3.75f},
      {2.175f, 2.5f},
      {2.175f, 2.5f},
      {3.1f,   15.f},
      {2.61f,  3.75f},
      {3.1f,   15.f},
      {2.61f,  5.f},
  }};

  cfg.dynamics_b1 = 1.2725f;
  cfg.dynamics_b2 = 0.7917f;

  cfg.rc_ch_mode_switch = RcReceiver::kSc;
  cfg.rc_ch_next_joint  = RcReceiver::kSh;

  return cfg;
}

