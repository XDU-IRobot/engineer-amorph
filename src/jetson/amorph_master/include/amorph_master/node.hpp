#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <librm.hpp>

#include "sbus_interface/msg/sbus.hpp"
#include "rm_referee_msgs/msg/remote_control.hpp"

#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include "pick_place_task/pick_place_task.hpp"
#include "pick_energy_unit_task/pick_energy_unit_task.hpp"

namespace amorph {

class AmorphMasterNode : public rclcpp::Node {
 public:
  explicit AmorphMasterNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions()                                       //
                                               .allow_undeclared_parameters(true)                      //
                                               .automatically_declare_parameters_from_overrides(true)  //
  );

  ~AmorphMasterNode();

 private:
  void InitUserInput();

  std::atomic<bool> planning_in_progress_{false};
  std::atomic<bool> executing_in_progress_{false};
  moveit_task_constructor_msgs::msg::Solution pending_solution_{};
  size_t current_step_{0};

  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
  rclcpp_action::Client<ExecuteTaskSolution>::SharedPtr execute_task_client_;

  struct {
    rclcpp::TimerBase::SharedPtr rpc_heartbeat{nullptr};
  } timers_{};
  struct {
    rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_remote{nullptr};
    rclcpp::Subscription<rm_referee_msgs::msg::RemoteControl>::SharedPtr vt_remote{nullptr};
  } subs_{};
  struct {
    PickPlaceTask pick_place_task{"pick_place_task"};
    PickEnergyUnitTask pick_energy_unit_task;
  } mtc_tasks_;
};

}  // namespace amorph