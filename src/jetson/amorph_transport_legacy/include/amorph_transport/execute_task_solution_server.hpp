#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "host_comm.hpp"
#include "trajectory_tracker.hpp"

namespace amorph {

class ExecuteTaskSolutionServer {
 public:
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTaskSolution>;

  ExecuteTaskSolutionServer(rclcpp::Node* node,
                            Stm32& stm32,
                            TrajectoryTracker<6>& trajectory_tracker);

 private:
  void Execute(std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node* node_;
  Stm32& stm32_;
  TrajectoryTracker<6>& trajectory_tracker_;

  rclcpp_action::Server<ExecuteTaskSolution>::SharedPtr server_;
};

}  // namespace amorph
