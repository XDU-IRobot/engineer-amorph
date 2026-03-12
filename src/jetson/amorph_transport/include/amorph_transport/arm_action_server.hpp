#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <string>
#include <vector>

#include "host_comm.hpp"
#include "trajectory_tracker.hpp"

namespace amorph {

class ArmActionServer {
 public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  ArmActionServer(rclcpp::Node* node,
                  std::string server_name,
                  std::vector<std::string> joint_names,
                  Stm32& stm32,
                  TrajectoryTracker<6>& trajectory_tracker);

 private:
  void Execute(std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node* node_;
  std::string server_name_;
  std::vector<std::string> joint_names_;
  Stm32& stm32_;
  TrajectoryTracker<6>& trajectory_tracker_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
};

}  // namespace amorph
