#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rpc/rpc.hpp"

namespace amorph {

class ArmControllerNode : public rclcpp::Node {
 public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit ArmControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void InitRpc();

 private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  RPC stm32_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
};

}  // namespace amorph