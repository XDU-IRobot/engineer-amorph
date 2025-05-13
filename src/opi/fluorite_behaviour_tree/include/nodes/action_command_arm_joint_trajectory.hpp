
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
          if (!goal_handle) {
            execution_result_ = rclcpp_action::ResultCode::ABORTED;
          } else {
            execution_result_ = rclcpp_action::ResultCode::UNKNOWN;
          }
        },
        [this](const typename ActionClient<fluorite_msgs::action::ArmTrajectoryCommand>::GoalHandleType::SharedPtr&
                   goal_handle,
               const std::shared_ptr<const fluorite_msgs::action::ArmTrajectoryCommand::Feedback> feedback) {
          // no feedback
        },
        [this](const typename ActionClient<fluorite_msgs::action::ArmTrajectoryCommand>::GoalHandleType::WrappedResult&
                   result) { execution_result_ = result.code; });
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    switch (execution_result_.load()) {
      case rclcpp_action::ResultCode::UNKNOWN: {
        return BT::NodeStatus::RUNNING;
      }
      case rclcpp_action::ResultCode::SUCCEEDED: {
        return BT::NodeStatus::SUCCESS;
      }
      case rclcpp_action::ResultCode::CANCELED:
      case rclcpp_action::ResultCode::ABORTED: {
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::FAILURE;  // just to suppress compiler warning, should never reach here
  }

  void onHalted() override {}

 private:
  std::atomic<rclcpp_action::ResultCode> execution_result_{rclcpp_action::ResultCode::UNKNOWN};
  rclcpp::Node::SharedPtr bt_node_;
};

#endif