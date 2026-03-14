#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <string>

#include "host_comm.hpp"

namespace amorph {

class GripperActionServer {
 public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;

  GripperActionServer(rclcpp::Node* node, std::string server_name, Stm32& stm32);

 private:
  void Execute(std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node* node_;
  std::string server_name_;
  Stm32& stm32_;

  rclcpp_action::Server<GripperCommand>::SharedPtr server_;
};

}  // namespace amorph
