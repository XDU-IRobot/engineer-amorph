
#pragma once

// ROS
#include <rclcpp/node.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include "mtc/mtc_task_interface.hpp"
#include "mtc/pick_place_task/params.hpp"

namespace moveit_task_constructor_demo {

// prepare a demo environment from ROS parameters under node
void SetupDemoScene(const pick_place_task::Params& params);

class PickPlaceTask final : public MtcTask<pick_place_task::Params> {
 public:
  explicit PickPlaceTask(const rclcpp::Node::SharedPtr& node, const std::string& task_name);
  ~PickPlaceTask() override = default;

  bool Init(const rclcpp::Node::SharedPtr& node) override;

  bool Plan(size_t max_solutions) override;

  bool Execute();

  auto& task() { return task_; }

 private:
  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;
};
}  // namespace moveit_task_constructor_demo
