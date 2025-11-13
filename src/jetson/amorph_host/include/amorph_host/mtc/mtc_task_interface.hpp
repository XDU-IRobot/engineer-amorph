
#pragma once

#include "ros_params_manager.hpp"

template <typename ParamType>
class MtcTask {
 public:
  explicit MtcTask(std::string task_name) : task_name_(std::move(task_name)) {}
  virtual ~MtcTask() = default;

  virtual bool Init(const rclcpp::Node::SharedPtr& node, const ParamType& params) = 0;

  virtual bool Plan(size_t max_solutions) = 0;

  virtual bool Execute() = 0;

 protected:
  void ReadParams() { params_manager_.Init(); }

 private:
  RosParamsManager<ParamType> params_manager_;
  std::string task_name_;
};
