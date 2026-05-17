#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rm_referee_msgs/msg/custom_robot_data.hpp>

#include <librm.hpp>

#include "host_comm.hpp"
#include "params_manager.hpp"
#include "ros_params.hpp"
#include "trajectory_tracker.hpp"
#include "arm_action_server.hpp"
#include "gripper_action_server.hpp"
#include "execute_task_solution_server.hpp"

namespace amorph {

class TransportNode : public rclcpp::Node {
 public:
  explicit TransportNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions()                                       //
                                               .allow_undeclared_parameters(true)                      //
                                               .automatically_declare_parameters_from_overrides(true)  //
  );

  void InitComm();

 private:
  void InitActionServers();

  ParamsManager<RosParams> params_manager_{this};

  rclcpp::TimerBase::SharedPtr stm32_tx_timer_{nullptr};

  std::unique_ptr<rm::hal::Can> comm_can_{nullptr};
  std::unique_ptr<Stm32> stm32_{nullptr};

  TrajectoryTracker<6> trajectory_tracker_{
      {-0.698132f, 0.349066f},     // joint 1 limits (j1_l)
      {1.362579f, 2.93337535f},    // joint 2 limits (j2_l)
      {-2.933375f, 0.0f},          // joint 3 limits (j3_l)
      {-3.14f, 3.14f},             // joint 4 limits (j4_l)
      {-1.745329f, 1.570796f},     // joint 5 limits (j5_l)
      {-3.14f, 3.14f}              // joint 6 limits (j6_l)
  };

  std::unique_ptr<ArmActionServer> arm_action_server_;
  std::unique_ptr<GripperActionServer> gripper_action_server_;
  std::unique_ptr<ExecuteTaskSolutionServer> execute_task_solution_server_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<rm_referee_msgs::msg::CustomRobotData>::SharedPtr custom_ctrl_sub_;
};

}  // namespace amorph