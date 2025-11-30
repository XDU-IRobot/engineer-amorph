#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rpc/rpc.hpp"
#include "params_manager.hpp"
#include "ros_params.hpp"

namespace amorph {

class TransportNode : public rclcpp::Node {
 public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

  explicit TransportNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions()                                       //
                                               .allow_undeclared_parameters(true)                      //
                                               .automatically_declare_parameters_from_overrides(true)  //
  );

  void InitRpc();

 private:
  void InitActionServers();

  ParamsManager<RosParams> params_manager_{rclcpp::Node::SharedPtr(this)};

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr arm_action_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_action_server_;
  RPC stm32_;

  void Execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void Execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);
};

}  // namespace amorph