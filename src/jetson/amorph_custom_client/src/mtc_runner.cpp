/**
 * @file mtc_runner.cpp
 * @brief MtcRunner 实现
 */

#include "amorph_custom_client/mtc_runner.hpp"
#include "amorph_custom_client/mtc/pick_energy_unit_task.hpp"

namespace amorph {

MtcRunner::MtcRunner(rclcpp::Node* node, const std::string& task_name)
    : node_(node),
      task_name_(task_name),
      task_(std::make_unique<PickEnergyUnitTask>(node, task_name)) {}

MtcRunner::~MtcRunner() = default;

void MtcRunner::run() {
  if (!task_) {
    if (plan_cb_) plan_cb_(false, "Task not initialized");
    return;
  }

  task_->SetObjectPose(object_pose_);
  task_->SetLiftDirection(lift_upward_);

  if (!task_->Setup()) {
    if (plan_cb_) plan_cb_(false, "MTC Setup 失败");
    return;
  }

  if (!task_->plan(10)) {
    if (plan_cb_) plan_cb_(false, "MTC 规划失败 (无可行解)");
    return;
  }

  if (plan_cb_) plan_cb_(true, "规划成功");
}

}  // namespace amorph
