#pragma once

// ROS
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "mtc_includes.hpp"

namespace amorph {

class PickEnergyUnitTask {
 public:
  PickEnergyUnitTask(rclcpp::Node* node, const std::string& task_name);
  ~PickEnergyUnitTask() = default;

  /// 设置能量单元在 world 系下的位姿（由外部调用者从 TF 或其他来源传入）
  void SetObjectPose(const geometry_msgs::msg::Pose& pose) { object_pose_ = pose; }

  /// 配置 MTC 流水线（需要先调用 SetObjectPose）
  bool Setup();

  bool plan(std::size_t max_solutions);

  bool execute();

  auto& task() const { return task_; }

  /// 设置 Lift Object 阶段的方向：true = 向上(z+1)，false = 向下(z-1)
  void SetLiftDirection(bool upward) { lift_upward_ = upward; }

 private:
  rclcpp::Node* node_;
  std::string task_name_;
  geometry_msgs::msg::Pose object_pose_;  // 由外部通过 SetObjectPose 传入
  moveit::task_constructor::TaskPtr task_;
  bool lift_upward_ = false;  // 默认向下 (z-1)
};

}  // namespace amorph
