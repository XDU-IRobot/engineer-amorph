/**
 * @file  ros_node.hpp
 * @brief 一个rclcpp::Node单例，供决策树节点使用
 */

#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "fluorite_msgs/action/arm_end_effector_cartesian_command.hpp"
#include "fluorite_msgs/action/arm_joint_command.hpp"
#include "fluorite_msgs/action/arm_trajectory_command.hpp"
#include "fluorite_msgs/srv/storage_system_command.hpp"
#include "fluorite_msgs/srv/sucker_command.hpp"

#include "action_client.hpp"

class RosNodeSingleton {
 public:
  static RosNodeSingleton &GetInstance() {
    static RosNodeSingleton instance;
    return instance;
  }

  rclcpp::Node::SharedPtr GetNode() { return node_; }

  ActionClient<fluorite_msgs::action::ArmEndEffectorCartesianCommand> arm_end_effector_cartesian_command_ac_;
  ActionClient<fluorite_msgs::action::ArmJointCommand> arm_joint_command_ac_;
  ActionClient<fluorite_msgs::action::ArmTrajectoryCommand> arm_trajectory_command_ac_;
  rclcpp::Client<fluorite_msgs::srv::StorageSystemCommand>::SharedPtr storage_system_command_client_;
  rclcpp::Client<fluorite_msgs::srv::SuckerCommand>::SharedPtr sucker_command_client_;

 private:
  RosNodeSingleton()
      : node_(rclcpp::Node::make_shared("fluorite_behaviour_tree")),
        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    executor_->add_node(node_);
    executor_thread_ = std::thread([&] { executor_->spin(); });

    // create action clients
    arm_end_effector_cartesian_command_ac_.Create(node_, "fluorite/arm/end_effector_cartesian_command");
    arm_joint_command_ac_.Create(node_, "fluorite/arm/joint_command");
    arm_trajectory_command_ac_.Create(node_, "fluorite/arm/trajectory_command");
    arm_end_effector_cartesian_command_ac_.CheckServerOnline();
    arm_joint_command_ac_.CheckServerOnline();
    arm_trajectory_command_ac_.CheckServerOnline();

    // create service clients
    storage_system_command_client_ =
        node_->create_client<fluorite_msgs::srv::StorageSystemCommand>("fluorite/arm/storage_system_command");
    sucker_command_client_ = node_->create_client<fluorite_msgs::srv::SuckerCommand>("fluorite/arm/sucker_system_command");
    while (!storage_system_command_client_->wait_for_service(std::chrono::milliseconds(100))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for service storage_system_command...");
    }
    while (!sucker_command_client_->wait_for_service(std::chrono::milliseconds(100))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for service sucker_system_command...");
    }
  }
  ~RosNodeSingleton() {
    executor_->cancel();
    executor_->remove_node(node_);
    executor_thread_.join();
    node_.reset();
    executor_.reset();
  }

  RosNodeSingleton(const RosNodeSingleton &) = delete;
  RosNodeSingleton &operator=(const RosNodeSingleton &) = delete;
  RosNodeSingleton(RosNodeSingleton &&) = delete;
  RosNodeSingleton &operator=(RosNodeSingleton &&) = delete;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic<bool> in_use_{false};
};

#endif  // ROS_NODE_HPP