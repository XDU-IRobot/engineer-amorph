#pragma once

#include "rc_forward.hpp"

namespace detail {
struct PDParams {
  float kp;
  float kd;
};
}  // namespace detail

#if defined(I_AM_LEFT_ARM)

#define CONFIG_LOWER_MOTOR_CAN hfdcan2
#define CONFIG_UPPER_MOTOR_CAN hfdcan1
#define CONFIG_HOST_COMM_ID 0x20

#define CONFIG_GRIPPER_MOTOR_ID 1

// clang-format off
#define CONFIG_MOTOR_CALIB_CONFIG \
    {true, 0.f},                                                 \
    {false, -rm::modules::Angle::FromDeg(90.f - 11.93f).rad()},  \
    {false, rm::modules::Angle::FromDeg(40.f - 11.93f).rad()},   \
    {false, 0.f},                                                \
    {true, 0.f},                                                 \
    {false, 0.f},                                                \
    {false, 0.f}
#define CONFIG_JOINT_LIMITS \
    /* 关节最小位置，关节最大位置（rad） */                            \
   {{{-0.698132f,   0.349066},                                    \
     {1.362579,     2.93337535},                                  \
     {-2.933375,    0.0},                                         \
     {-M_PI,        M_PI},                                        \
     {-M_PI / 2.f,  M_PI / 2.f},                                  \
     {-M_PI,        M_PI},                                        \
     {-M_PI,        M_PI}}}
#define CONFIG_TRAJECTORY_LIMITS \
    /* 关节最大速度（rad/s），关节最大加速度（rad/s^2） */                     \
   {{{2.175f,  3.75f},                                                   \
     {2.175f,  2.5f},                                                    \
     {2.175f,  2.5f},                                                    \
     {3.1f,    15.f},                                                    \
     {2.61f,   3.75f},                                                   \
     {3.1f,    15.f},                                                    \
     {2.61f,   5.f}}}
//clang-format on

constexpr detail::PDParams CONFIG_MOTOR_PD_PARAMS[6] = {  //
    {60.f, 25.f},
    {120.f, 35.f},
    {100.f, 35.f},
    {35.f, 15.f},
    {35.f, 15.f},
    {30.f, 10.f},
};

constexpr auto CONFIG_RC_CH_MODE_SWITCH = RcReceiver::kSb;    ///< 遥控器上哪个通道用来切换模式
constexpr auto CONFIG_RC_CH_NEXT_JOINT = RcReceiver::kSh;     ///< 遥控器上哪个通道用来在手动模式下切换控制的关节

constexpr float CONFIG_DYNAMICS_PARAM_B1 = 12.470911f;         ///< 动力学模型的b1参数
constexpr float CONFIG_DYNAMICS_PARAM_B2 = 7.758989f;          ///< 动力学模型的b2参数

#elif defined(I_AM_RIGHT_ARM)

#define CONFIG_LOWER_MOTOR_CAN hfdcan1
#define CONFIG_UPPER_MOTOR_CAN hfdcan2
#define CONFIG_HOST_COMM_ID 0x30

#define CONFIG_GRIPPER_MOTOR_ID 3

// clang-format off
#define CONFIG_MOTOR_CALIB_CONFIG \
    {true, 0.f},                                                 \
    {true, rm::modules::Angle::FromDeg(90.f - 11.93f).rad()},  \
    {true, -rm::modules::Angle::FromDeg(40.f - 11.93f).rad()},   \
    {false, 0.f},                                                \
    {false, 0.f},                                                 \
    {false, 0.f},                                                \
    {false, 0.f}
#define CONFIG_JOINT_LIMITS \
    /* 关节最小位置（rad），关节最大位置（rad） */                     \
   {{{-0.698132f,   0.349066},                                    \
     {1.362579,     2.93337535},                                  \
     {-2.933375,    -0.48},                                         \
     {-M_PI,        M_PI},                                        \
     {-M_PI / 2.f,  M_PI / 2.f},                                  \
     {-M_PI,        M_PI},                                        \
     {-M_PI,        M_PI}}}
#define CONFIG_TRAJECTORY_LIMITS \
    /* 关节最大速度（rad/s），关节最大加速度（rad/s^2） */                     \
   {{{2.175f,  3.75f},                                                   \
     {2.175f,  2.5f},                                                    \
     {2.175f,  2.5f},                                                    \
     {3.1f,    15.f},                                                    \
     {2.61f,   3.75f},                                                   \
     {3.1f,    15.f},                                                    \
     {2.61f,   5.f}}}
//clang-format on

constexpr detail::PDParams CONFIG_MOTOR_PD_PARAMS[6] = {  //
  {60.f, 25.f},
  {140.f, 40.f},
  {120.f, 40.f},
  {35.f, 15.f},
  {35.f, 10.f},
  {30.f, 10.f},
};

constexpr auto CONFIG_RC_CH_MODE_SWITCH = RcReceiver::kSc;    ///< 遥控器上哪个通道用来切换模式
constexpr auto CONFIG_RC_CH_NEXT_JOINT = RcReceiver::kSh;     ///< 遥控器上哪个通道用来在手动模式下切换控制的关节

constexpr float CONFIG_DYNAMICS_PARAM_B1 = 1.2725f;         ///< 动力学模型的b1参数
constexpr float CONFIG_DYNAMICS_PARAM_B2 = 0.7917f;          ///< 动力学模型的b2参数

#endif
