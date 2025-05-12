
#ifndef NODES_ACTION_COMMAND_ARM_JOINT_TRAJECTORY_HPP
#define NODES_ACTION_COMMAND_ARM_JOINT_TRAJECTORY_HPP

#include <atomic>

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
    auto action_client = RosNodeSingleton::GetInstance().arm_trajectory_command_ac_;
    if (!action_client.CheckServerOnline()) {
      return BT::NodeStatus::FAILURE;
    }

    BT::Optional<trajectory_msgs::msg::JointTrajectory> traj =
        getInput<trajectory_msgs::msg::JointTrajectory>("trajectory");
    if (!traj) {
      throw BT::RuntimeError("missing required input [traj]: ", traj.error());
    }

    const auto action_goal = [&] {
      auto goal = std::make_shared<fluorite_msgs::action::ArmTrajectoryCommand::Goal>();
      goal->trajectory = *traj;
      return goal;
    }();

    action_client.AsyncSendGoal(
        action_goal,
        [this](const typename ActionClient<fluorite_msgs::action::ArmTrajectoryCommand>::GoalHandleType::SharedPtr
                   goal_handle) {
          // server那边无条件accept，不用处理被拒绝的逻辑
          RCLCPP_INFO(bt_node_->get_logger(), "Goal accepted");
        },
        [this](const typename ActionClient<fluorite_msgs::action::ArmTrajectoryCommand>::GoalHandleType::SharedPtr&
                   goal_handle,
               const std::shared_ptr<const fluorite_msgs::action::ArmTrajectoryCommand::Feedback> feedback) {
          RCLCPP_INFO(bt_node_->get_logger(), "Feedback received");
        },
        [this](const typename ActionClient<fluorite_msgs::action::ArmTrajectoryCommand>::GoalHandleType::WrappedResult&
                   result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(bt_node_->get_logger(), "Action succeeded");
          } else {
            RCLCPP_ERROR(bt_node_->get_logger(), "Action failed");
          }
          execution_finished_ = true;
        });
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!execution_finished_) {
      return BT::NodeStatus::RUNNING;
    }
    execution_finished_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}

 private:
  std::atomic<bool> execution_finished_{false};
  rclcpp::Node::SharedPtr bt_node_;
};

#endif