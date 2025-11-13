
#pragma once

#include "ros_params_manager.hpp"

template <typename ParamType>
class MtcTask {
 public:
  explicit MtcTask(const rclcpp::Node::SharedPtr& node, std::string task_name)
      : params_manager_(node), task_name_(std::move(task_name)) {}
  virtual ~MtcTask() = default;

  virtual bool Init(const rclcpp::Node::SharedPtr& node) = 0;

  virtual bool Plan(size_t max_solutions) = 0;

 protected:
  void ReadParams() { params_manager_->Init(); }

 private:
  RosParamsManager<ParamType> params_manager_;
  std::string task_name_;
};
