/**
 * @file    ros_params_manager.hpp
 * @brief   用boost::pfr反射处理ros参数，防止写一大堆boilerplate code
 */

#pragma once

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

namespace amorph {

template <class ParamsStructType>
class ParamsManager {
 public:
  ParamsManager() = delete;
  ~ParamsManager() = default;

  explicit ParamsManager(rclcpp::Node *node) : node_(node) {}

  const ParamsStructType &data() const { return ros_params_; }

  void Init() {
    // 递归加载参数
    LoadParams("", ros_params_);

    // 递归统计最大长度
    int max_name_length = 0;
    int max_value_length = 0;
    CalcMaxLength("", ros_params_, max_name_length, max_value_length);
    RCLCPP_INFO(node_->get_logger(), "ROSParamsManager: params ok");
    RCLCPP_INFO(node_->get_logger(),
                "================================================================================");
    PrintParams("", ros_params_, max_name_length, max_value_length);
    RCLCPP_INFO(node_->get_logger(),
                "================================================================================");

    // 启动参数服务器
    param_cb_handle_ =
        node_->add_on_set_parameters_callback([this](auto &&arg) -> rcl_interfaces::msg::SetParametersResult {
          return ParameterCallback(std::forward<decltype(arg)>(arg));
        });
  }

 private:
  // 判断是否为std::vector
  template <typename T>
  struct is_std_vector : std::false_type {};
  template <typename T, typename Alloc>
  struct is_std_vector<std::vector<T, Alloc>> : std::true_type {};

  // 判断是否为可递归反射的结构体类型（聚合且非string非vector）
  template <typename T>
  inline static constexpr bool is_reflectable_struct_v =
      std::is_aggregate_v<T> && !std::is_same_v<T, std::string> && !is_std_vector<T>::value;

  // 递归加载参数
  template <typename T>
  void LoadParams(const std::string &prefix, T &obj) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        LoadParams(full_name, field);
      });
    } else {
      node_->get_parameter(prefix, obj);
    }
  }

  // 递归统计最大长度
  template <typename T>
  void CalcMaxLength(const std::string &prefix, T &obj, int &max_name_length, int &max_value_length) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        CalcMaxLength(full_name, field, max_name_length, max_value_length);
      });
    } else {
      max_name_length = std::max(max_name_length, static_cast<int>(prefix.size()));
      std::ostringstream oss;
      oss << obj;
      max_value_length = std::max(max_value_length, static_cast<int>(oss.str().size()));
    }
  }

  // 递归打印参数
  template <typename T>
  void PrintParams(const std::string &prefix, T &obj, int max_name_length, int max_value_length) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        PrintParams(full_name, field, max_name_length, max_value_length);
      });
    } else {
      std::ostringstream oss;
      oss << obj;
      std::string fmt = "%-" + std::to_string(max_name_length) + "s     %-" + std::to_string(max_value_length) + "s";
      RCLCPP_INFO(node_->get_logger(), fmt.c_str(), prefix.c_str(), oss.str().c_str());
    }
  }

  // 递归参数回调
  template <typename T>
  void UpdateParamByName(const std::string &prefix, T &obj, const rclcpp::Parameter &parameter) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        UpdateParamByName(full_name, field, parameter);
      });
    } else {
      if (prefix == parameter.get_name()) {
        RCLCPP_INFO(node_->get_logger(), "ROSParamsManager: parameter %s has been updated to %s", prefix.c_str(),
                    parameter.value_to_string().c_str());
        node_->get_parameter(prefix, obj);
      }
    }
  }

  /**
   * @brief    参数服务器回调函数
   */
  auto ParameterCallback(const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
    for (const auto &parameter : parameters) {
      UpdateParamByName("", ros_params_, parameter);
    }
    return rcl_interfaces::msg::SetParametersResult();
  }

 private:
  rclcpp::Node *node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  ParamsStructType ros_params_{};
};

}  // namespace amorph