
// 使用tinyfsm之后，就没办法再使用oop的各种编程范式来写代码了，因为状态机类没有对象
// 这里被迫创建了几个全局变量（定义都在fsm.cc里），用来存储状态机的外部依赖
// 如果以后找到了更好的实现方式，可以考虑重构

#include <mutex>
#include <atomic>
#include <memory>
#include <type_traits>

#include <tinyfsm.hpp>
#include <librm.hpp>

// ros
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "fluorite_msgs/msg/arm_state.hpp"
#include "arm/pneumatic.hpp"
#include "arm/dynamics.hpp"
#include "arm/pid_trajectory_tracker.hpp"
#include "arm/moveit_helper.hpp"
#include "common/logger.hpp"

namespace fluorite {
/**
 * @brief   机器人状态向量，由节点的各个传感器数据回调函数更新
 */
extern fluorite_msgs::msg::ArmState state;

/**
 * @brief   执行器（电机、气泵、电磁阀）
 */
struct Actuator {
  Actuator(rm::device::DirectDriveMotor &&base_yaw, rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&yaw,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&pitch_1,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&pitch_2,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&end_yaw_1,
           rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> &&end_pitch, rm::device::GM6020 &&end_yaw_2,
           SuckerSystem &&sucker_system, StorageSystem &&storage_system)
      : base_yaw(std::move(base_yaw)),
        yaw(std::move(yaw)),
        pitch_1(std::move(pitch_1)),
        pitch_2(std::move(pitch_2)),
        end_yaw_1(std::move(end_yaw_1)),
        end_pitch(std::move(end_pitch)),
        end_yaw_2(std::move(end_yaw_2)),
        sucker_system(std::move(sucker_system)),
        storage_system(std::move(storage_system)) {}
  rm::device::DirectDriveMotor base_yaw;
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> yaw, pitch_1, pitch_2, end_yaw_1, end_pitch;
  rm::device::GM6020 end_yaw_2;
  SuckerSystem sucker_system;
  StorageSystem storage_system;
};
extern std::unique_ptr<Actuator> actuator;

/**
 * @brief   控制器（算法对象）
 */
struct Controller {
  using PosPID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;
  using PosRingPID = rm::modules::algorithm::RingPID<rm::modules::algorithm::PIDType::kPosition>;

  Controller(PidTrajectoryTracker<6> &&ptt, PosPID &&base_yaw_speed_pid, PosRingPID &&base_yaw_position_pid,
             std::unique_ptr<tf2_ros::Buffer> &&tf_buffer, rclcpp::Node *ros_node)
      : trajectory_tracker{std::move(ptt)},
        base_yaw_speed_pid{std::move(base_yaw_speed_pid)},
        base_yaw_position_pid{std::move(base_yaw_position_pid)},
        tf_buffer(std::move(tf_buffer)),
        moveit_helper{ros_node} {}
  PidTrajectoryTracker<6> trajectory_tracker;
  PosPID base_yaw_speed_pid;
  PosRingPID base_yaw_position_pid;

  DynamicModel dynamic_model{};  ///< 逆动力学模型

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  MoveitHelper moveit_helper;
  std::mutex ik_calculation_mutex;
  rm::core::ThreadPool ik_calculation_thread_pool_{1};
};
extern std::unique_ptr<Controller> controller;
}  // namespace fluorite