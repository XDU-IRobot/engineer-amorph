/**
 * @file mtc_runner.hpp
 * @brief MTC 任务执行器 —— 封装 PickEnergyUnitTask 的生命周期
 *
 * 不依赖 Qt，仅依赖 ROS + MTC。
 * OperatorPanel 在用户按下 Plan 按钮时创建 runner 并启动。
 */

#pragma once

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <functional>
#include <memory>

namespace amorph {

class PickEnergyUnitTask;

/// 规划完成时的回调（success, message）
using PlanCallback = std::function<void(bool success, const std::string& message)>;

class MtcRunner {
 public:
  MtcRunner(rclcpp::Node* node, const std::string& task_name);
  ~MtcRunner();

  /// 设置物体位姿（必须在 run() 前调用）
  void setObjectPose(const geometry_msgs::msg::Pose& pose) { object_pose_ = pose; }

  /// 设置 Lift 方向
  void setLiftDirection(bool upward) { lift_upward_ = upward; }

  /// 设置规划完成回调
  void setPlanCallback(PlanCallback cb) { plan_cb_ = std::move(cb); }

  /// 执行完整的 Setup → plan 流程（同步、阻塞）
  /// 完成后通过 plan_cb_ 通知结果
  void run();

 private:
  rclcpp::Node* node_;
  std::string task_name_;
  geometry_msgs::msg::Pose object_pose_;
  bool lift_upward_ = false;
  PlanCallback plan_cb_;
  std::unique_ptr<PickEnergyUnitTask> task_;
};

}  // namespace amorph
