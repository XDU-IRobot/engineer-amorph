/**
 * @file    ros_params_manager.hpp
 * @brief   用boost::pfr反射处理ros参数，防止写一大堆boilerplate code
 */

#ifndef COMMON_ROS_PARAMS_MANAGER_HPP
#define COMMON_ROS_PARAMS_MANAGER_HPP

#include <functional>
#include <memory>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <boost/pfr.hpp>
#include <boost/pfr/core_name.hpp>

/**
 * @brief   打印vector
 */
template <typename Tp>
std::ostream &operator<<(std::ostream &os, const std::vector<Tp> &vec) {
  os << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    os << vec[i];
    if (i != vec.size() - 1) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

template <class ParamsStructType>
class RosParamsManager {
 public:
  RosParamsManager() = delete;
  ~RosParamsManager() = default;

  RosParamsManager(rclcpp::Node *node) : node_(node) {}

  const ParamsStructType &data() const { return ros_params_; }

  void Init() {
    // 遍历ros_params_结构体的每个字段，从传入的节点参数中读入对应的参数
    boost::pfr::for_each_field_with_name(
        ros_params_, [&](std::string_view name, auto &field) { node_->get_parameter(std::string(name), field); });

    // 把所有参数打印出来
    int max_name_length = 0, max_value_length = 0;
    boost::pfr::for_each_field_with_name(ros_params_, [&](std::string_view name, auto &field) {
      std::ostringstream oss;
      oss << field;
      max_name_length = std::max(max_name_length, static_cast<int>(name.size()));
      max_value_length = std::max(max_value_length, static_cast<int>(oss.str().size()));
    });
    RCLCPP_INFO(node_->get_logger(), "ROSParamsManager: params ok");
    RCLCPP_INFO(node_->get_logger(),
                "================================================================================");
    boost::pfr::for_each_field_with_name(ros_params_, [&](std::string_view name, auto &field) {
      std::ostringstream oss;
      oss << field;
      std::string fmt = "%-" + std::to_string(max_name_length) + "s     %-" + std::to_string(max_value_length) + "s";
      RCLCPP_INFO(node_->get_logger(), fmt.c_str(), name.data(), oss.str().c_str());
    });
    RCLCPP_INFO(node_->get_logger(),
                "================================================================================");

    /** 启动参数服务器 **/
    // param_cb_handle_ = node_->add_on_set_parameters_callback(
    //     std::bind(&RosParamsManager::ParameterCallback, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief    参数服务器回调函数
   */
  auto ParameterCallback(const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
    for (const auto &parameter : parameters) {
      boost::pfr::for_each_field_with_name(ros_params_, [&](std::string_view name, auto &field) {
        if (name == parameter.get_name()) {
          RCLCPP_INFO(node_->get_logger(), "ROSParamsManager: parameter %s has been updated to %s", name.data(),
                      parameter.value_to_string().c_str());
          node_->get_parameter(std::string(name), field);
        }
      });
    }
    return rcl_interfaces::msg::SetParametersResult();
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  ParamsStructType ros_params_{};
};

#endif  // COMMON_ROS_PARAMS_MANAGER_HPP