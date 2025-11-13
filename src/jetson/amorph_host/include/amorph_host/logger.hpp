#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace detail {
class NonCopyable {
 public:
  NonCopyable(const NonCopyable&) = delete;
  NonCopyable& operator=(const NonCopyable&) = delete;

 protected:
  NonCopyable() = default;
  ~NonCopyable() = default;
};
}  // namespace detail

class RosLogger : detail::NonCopyable {
 public:
  static RosLogger& GetInstance() {
    static RosLogger instance;
    return instance;
  }

  void Init(rclcpp::Node::SharedPtr node) { node_ = std::move(node); }

  template <typename... Args>
  inline void Debug(const char* format, Args&&... args) {
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  inline void Info(const char* format, Args&&... args) {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  inline void Warn(const char* format, Args&&... args) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  inline void Error(const char* format, Args&&... args) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  inline void Fatal(const char* format, Args&&... args) {
    if (node_) {
      RCLCPP_FATAL(node_->get_logger(), format, std::forward<Args>(args)...);
    }
  }

 private:
  RosLogger() = default;
  rclcpp::Node::SharedPtr node_;
};

#endif  // COMMON_LOGGER_HPP