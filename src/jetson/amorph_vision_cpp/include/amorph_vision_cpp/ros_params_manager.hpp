/**
 * @file    ros_params_manager.hpp
 * @brief   用boost::pfr反射处理ros参数，防止写一大堆boilerplate code
 */

#pragma once

#include <algorithm>
#include <functional>
#include <sstream>
#include <string_view>
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

namespace amorph::vision {

template <class ParamsStructType>
class ParamsManager {
 public:
  ParamsManager() = delete;
  ~ParamsManager() = default;

  // 用引用而不是 SharedPtr：节点在构造期无法拿到 shared_from_this()
  explicit ParamsManager(rclcpp::Node &node) : node_(node) {}

  const ParamsStructType &data() const { return ros_params_; }

  void Init() {
    // 递归加载参数
    LoadParams("", ros_params_);

    // 对齐打印，便于一眼扫完所有参数
    int max_name_length = 0;
    CalcMaxNameLength("", ros_params_, max_name_length);
    RCLCPP_INFO(node_.get_logger(), "ROSParamsManager: params ok");
    PrintParams("", ros_params_, max_name_length);

    // 启动参数服务器
    param_cb_handle_ =
        node_.add_on_set_parameters_callback([this](auto &&arg) -> rcl_interfaces::msg::SetParametersResult {
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
  //
  // 用 declare_parameter 而不是 get_parameter：参数在节点里只声明一次，
  // 默认值就是结构体里的初始值，外部 launch / YAML 的覆盖值优先。
  template <typename T>
  void LoadParams(const std::string &prefix, T &obj) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        LoadParams(full_name, field);
      });
    } else {
      obj = node_.declare_parameter(prefix, obj);
    }
  }

  // 递归统计参数名最大长度，用于对齐
  template <typename T>
  void CalcMaxNameLength(const std::string &prefix, T &obj, int &max_name_length) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        CalcMaxNameLength(full_name, field, max_name_length);
      });
    } else {
      max_name_length = std::max(max_name_length, static_cast<int>(prefix.size()));
    }
  }

  // 递归打印参数
  template <typename T>
  void PrintParams(const std::string &prefix, T &obj, int max_name_length) {
    if constexpr (is_reflectable_struct_v<T>) {
      boost::pfr::for_each_field_with_name(obj, [&](std::string_view name, auto &field) {
        std::string full_name = prefix.empty() ? std::string(name) : prefix + "." + std::string(name);
        PrintParams(full_name, field, max_name_length);
      });
    } else {
      std::ostringstream oss;
      oss << std::boolalpha << obj;
      const std::string fmt = "%-" + std::to_string(max_name_length) + "s     %s";
      RCLCPP_INFO(node_.get_logger(), fmt.c_str(), prefix.c_str(), oss.str().c_str());
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
        RCLCPP_INFO(node_.get_logger(), "ROSParamsManager: parameter %s has been updated to %s", prefix.c_str(),
                    parameter.value_to_string().c_str());
        node_.get_parameter(prefix, obj);
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
    // successful 默认为 false，不显式置 true 会拒绝所有参数设置
    // （包括 rclcpp 内部给 TF broadcaster 设的 qos_overrides.*）
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

 private:
  rclcpp::Node &node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  ParamsStructType ros_params_{};
};

}  // namespace amorph::vision
