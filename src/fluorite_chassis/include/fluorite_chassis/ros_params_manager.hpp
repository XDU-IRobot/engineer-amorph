/**
 * @file    ros_params_handler.hpp
 * @brief   用boost::pfr反射处理ros参数，防止写一大堆boilerplate code
 */

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

  RosParamsManager(rclcpp::Node::SharedPtr &node) : node_(node) {}
  RosParamsManager(rclcpp::Node::SharedPtr &&node) : node_(std::move(node)) {}
  RosParamsManager(rclcpp::Node *node) : node_(node->shared_from_this()) {}

  const ParamsStructType &data() const { return ros_params_; }

  void Init() {
    // 尝试拿到节点的shared_ptr
    rclcpp::Node::SharedPtr node{nullptr};
    try {
      node = node_.lock();
    } catch (const std::bad_weak_ptr &e) {  // this should never happen though...
      throw std::runtime_error("ROSParamsManager: node is expired, return");
      return;
    }

    // 遍历ros_params_结构体的每个字段，从传入的节点参数中读入对应的参数
    boost::pfr::for_each_field_with_name(
        ros_params_, [node](std::string_view name, auto &field) { node->get_parameter(std::string(name), field); });

    // 把所有参数打印出来
    int max_name_length = 0, max_value_length = 0;
    boost::pfr::for_each_field_with_name(ros_params_, [&](std::string_view name, auto &field) {
      std::ostringstream oss;
      oss << field;
      max_name_length = std::max(max_name_length, static_cast<int>(name.size()));
      max_value_length = std::max(max_value_length, static_cast<int>(oss.str().size()));
    });
    RCLCPP_INFO(node->get_logger(), "ROSParamsManager: params ok");
    RCLCPP_INFO(node->get_logger(), "================================================================================");
    boost::pfr::for_each_field_with_name(ros_params_, [&](std::string_view name, auto &field) {
      std::ostringstream oss;
      oss << field;
      std::string fmt = "%-" + std::to_string(max_name_length) + "s     %-" + std::to_string(max_value_length) + "s";
      RCLCPP_INFO(node->get_logger(), fmt.c_str(), name.data(), oss.str().c_str());
    });
    RCLCPP_INFO(node->get_logger(), "================================================================================");

    /** 启动参数服务器 **/
    param_cb_handle_ = node->add_on_set_parameters_callback(
        std::bind(&RosParamsManager::ParameterCallback, node.get(), std::placeholders::_1));
  }

 private:
  /**
   * @brief    参数服务器回调函数
   */
  auto ParameterCallback(const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
    for (const auto &parameter : parameters) {
      // 尝试拿到节点的shared_ptr
      // 还能进这个callback说明节点肯定还没释放，可以放心拿shared_ptr，所以没有像上面一样try-catch
      rclcpp::Node::SharedPtr node = node_.lock();

      boost::pfr::for_each_field_with_name(ros_params_, [&parameter, node](std::string_view name, auto &field) {
        if (name == parameter.get_name()) {
          RCLCPP_INFO(node->get_logger(), "ROSParamsManager: parameter %s has been updated to %s", name.data(),
                      parameter.value_to_string().c_str());
          node->get_parameter(std::string(name), field);
        }
      });
    }
    return rcl_interfaces::msg::SetParametersResult();
  }

 private:
  rclcpp::Node::WeakPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  ParamsStructType ros_params_{};
};