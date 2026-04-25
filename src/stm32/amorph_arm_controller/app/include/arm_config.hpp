/**
 * @file  arm_config.hpp
 * @brief 机械臂实例参数描述结构体
 *
 * 使用方式：
 *   - 调用 MakeLeftArmConfig / MakeRightArmConfig 获取预设参数
 *   - 向构造函数传入已创建好的 CAN 总线接口指针
 *   - 把得到的 ArmConfig 传给 ArmInstance 构造函数
 */

#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include <librm.hpp>

#include "rc_forward.hpp"

namespace detail {
struct PDParams {
  float kp;
  float kd;
};
}  // namespace detail

/**
 * @brief 描述单个机械臂实例所需的全部硬件资源指针和控制参数
 *
 * CAN 总线接口对象由调用方（main.cc 或 SharedResources）负责创建和生命期管理，
 * 这里只保存非所有权指针，从而支持多个 ArmInstance 共享同一根 CAN 总线。
 */
struct ArmConfig {
  // ---- 通信资源（非所有权指针） ----
  rm::hal::CanInterface* lower_motor_can{nullptr};  ///< 下臂电机 CAN 接口
  rm::hal::CanInterface* upper_motor_can{nullptr};  ///< 上臂电机 CAN 接口
  size_t host_comm_id{0};       ///< 此臂在通信总线上发送反馈帧的 CAN ID
  uint8_t gripper_motor_id{1};  ///< 夹爪电机 ID（LkMotorMS 用）

  // ---- 电机配置 ----
  rm::device::DmMotorMit::Settings dm_settings[6];           ///< j1~j6 达妙电机参数
  rm::modules::MotorCalibrator::MotorConfig calibrators[7];  ///< 7 关节校准配置

  // ---- 控制参数 ----
  detail::PDParams pd_params[6];  ///< j1~j6 PD 增益

  // ---- 运动学 / 动力学 ----
  std::array<std::pair<float, float>, 7> joint_limits;       ///< {min, max}（逻辑坐标，rad）
  std::array<std::pair<float, float>, 7> trajectory_params;  ///< {最大速度, 最大加速度}
  float dynamics_b1{0.f};                                    ///< 动力学参数 b1
  float dynamics_b2{0.f};                                    ///< 动力学参数 b2

  // ---- 遥控器通道映射 ----
  RcReceiver::Channel rc_ch_mode_switch{RcReceiver::kSb};  ///< 模式切换拨杆通道
  RcReceiver::Channel rc_ch_next_joint{RcReceiver::kSh};   ///< 切换关节拨杆通道
};

/**
 * @brief 构造左臂参数（电机 CAN 总线由调用者传入）
 * @param lower_can 下臂三轴电机（j1/j2/j3）所在的 CAN 总线
 * @param upper_can 上臂三轴电机（j4/j5/j6）及夹爪所在的 CAN 总线
 */
ArmConfig MakeLeftArmConfig(rm::hal::CanInterface& lower_can, rm::hal::CanInterface& upper_can);

/**
 * @brief 构造右臂参数（电机 CAN 总线由调用者传入）
 * @param lower_can 下臂三轴电机（j1/j2/j3）所在的 CAN 总线
 * @param upper_can 上臂三轴电机（j4/j5/j6）及夹爪所在的 CAN 总线
 */
ArmConfig MakeRightArmConfig(rm::hal::CanInterface& lower_can, rm::hal::CanInterface& upper_can);

