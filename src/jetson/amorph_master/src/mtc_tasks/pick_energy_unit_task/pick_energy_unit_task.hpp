#pragma once

// ROS
#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mtc_includes.hpp"

namespace amorph {

class PickEnergyUnitTask {
 public:
  PickEnergyUnitTask(rclcpp::Node* node, const std::string& task_name);
  ~PickEnergyUnitTask() = default;

  /// 内部通过 TF 查找 world->energy_unit_1_smoothed 变换来获取物体位姿
  bool Setup();

  bool plan(std::size_t max_solutions);

  bool execute();

  auto& task() const { return task_; }

 private:
  rclcpp::Node* node_;
  std::string task_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  moveit::task_constructor::TaskPtr task_;
};

}  // namespace amorph
