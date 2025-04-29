
#ifndef NODES_ACTION_COMMAND_ARM_JOINT_TRAJECTORY_HPP
#define NODES_ACTION_COMMAND_ARM_JOINT_TRAJECTORY_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "moveit.hpp"
#include "ros_node.hpp"

#include "fluorite_msgs/action/arm_trajectory_command.hpp"

class ActionCommandArmJointTrajectory : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionCommandArmJointTrajectory";

  ActionCommandArmJointTrajectory(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config), bt_node_{RosNodeSingleton::GetInstance().GetNode()} {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
    };
  }

  BT::NodeStatus onStart() override {
    BT::Optional<trajectory_msgs::msg::JointTrajectory> traj =
        getInput<trajectory_msgs::msg::JointTrajectory>("trajectory");
    if (!traj) {
      throw BT::RuntimeError("missing required input [traj]: ", traj.error());
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const auto traj_str = JsonBuilder::BuildJson(getInput<trajectory_msgs::msg::JointTrajectory>("trajectory").value());
    // RCLCPP_INFO(bt_node_->get_logger(), "Sending trajectory: %s", traj_str.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}

 private:
  rclcpp::Node::SharedPtr bt_node_;
};

#endif