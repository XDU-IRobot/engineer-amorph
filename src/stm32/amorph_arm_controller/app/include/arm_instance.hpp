/**
 * @file  arm_instance.hpp
 * @brief 单个机械臂实例的完整封装
 *
 * ArmInstance 包含该臂专属的所有对象：
 *   - Actuator（所有电机）
 *   - ArmController<7>（轨迹规划与控制器）
 *   - Host（与上位机的 CAN 通信）
 *   - DeviceManager（设备在线状态监测���
 *   - fsm::Arm 及全部 FSM 状态实例
 *
 * 共享硬件（IMU、RC、蜂鸣器、LED、通信 CAN）通过 SharedResources& 传入。
 *
 * 扩展方法：
 *   - 单臂：实例化一个 ArmInstance 并在主循环中调用 Tick()
 *   - 双臂：实例化两个 ArmInstance，各自拥有独立的状态机和电机对象，
 *           并在主循环中依次调用各自的 Tick()
 */

#pragma once

#include <etl/string.h>
#include <librm.hpp>

#include "arm_config.hpp"
#include "fsm.hpp"           // 包含 Arm、所有 state::* 类型，以及 shared_resources.hpp

/**
 * @brief 构造轨迹限制器数组（将 ArmConfig 中的 {max_vel, max_acc} 对转为 TrajectoryLimiter）
 */
inline std::array<rm::modules::TrajectoryLimiter, 7> MakeTrajectoryLimiters(
    const std::array<std::pair<float, float>, 7>& params) {
  return {
      rm::modules::TrajectoryLimiter{params[0].first, params[0].second},
      rm::modules::TrajectoryLimiter{params[1].first, params[1].second},
      rm::modules::TrajectoryLimiter{params[2].first, params[2].second},
      rm::modules::TrajectoryLimiter{params[3].first, params[3].second},
      rm::modules::TrajectoryLimiter{params[4].first, params[4].second},
      rm::modules::TrajectoryLimiter{params[5].first, params[5].second},
      rm::modules::TrajectoryLimiter{params[6].first, params[6].second},
  };
}

/**
 * @brief 单个机械臂实例
 */
struct ArmInstance {
  // ---- 每臂独有的控制对象 ----
  Actuator         actuator;        ///< 所有关节电机
  ArmController<7> arm_controller;  ///< 轨迹规划 + 控制输出计算
  Host             host;            ///< 与上位机通信

  rm::device::DeviceManager<10> device_manager;         ///< 管理该臂所有电机的在线状态
  rm::device::DeviceManager<1>  host_connection_watcher; ///< 监视上位机连接状态
  etl::string<512> status_str{""};                      ///< 描述当前设备状态的字符串

  // ---- 遥控器通道映射（从 ArmConfig 复制） ----
  RcReceiver::Channel rc_ch_mode_switch;
  RcReceiver::Channel rc_ch_next_joint;

  // ---- FSM：独立拥有全部状态实例 ----
  fsm::Arm arm_fsm;

  fsm::state::NoForce              state_no_force;
  fsm::state::Manual               state_manual;
  fsm::state::Idle                 state_idle;
  fsm::state::HostControl          state_host_control;
  fsm::state::CustomCtrl           state_custom_ctrl;
  fsm::state::CartesianCustomCtrl  state_cartesian_custom_ctrl;
  fsm::state::LongJointCalibrating state_long_joint_calibrating;
  fsm::state::Fault                state_fault;

 private:
  // etl::fsm::set_states 可能持有此数组地址，必须与 FSM 同生命期
  etl::ifsm_state* state_list_[8]{};

 public:
  /**
   * @brief 构造函数
   * @param config  此臂的全部参数（CAN 指针需已填入）
   * @param shared  全系统共享资源
   */
  explicit ArmInstance(const ArmConfig& config, SharedResources& shared);

  /**
   * @brief 初始化（设备命名、回调注册）
   *        电机 CAN 总线的 SetFilter + Begin 由调用方（main.cc）负责。
   * @param shared  全系统共享资源
   */
  void Init(SharedResources& shared);

  /**
   * @brief 500 Hz 控制循环节拍（IMU 数据从 SharedResources 内部读取）
   * @note  调用前须已调用 SharedResources::imu.Update()
   */
  void Tick();

  /**
   * @brief 50 Hz 慢速节拍：向上位机发送反馈
   */
  void TickSlow();
};



