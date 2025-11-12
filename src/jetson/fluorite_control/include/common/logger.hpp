#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace fluorite {

template <typename... Args>
inline void debug(const char* format, Args&&... args) {
  RCLCPP_DEBUG(rclcpp::get_logger(""), format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(const char* format, Args&&... args) {
  RCLCPP_INFO(rclcpp::get_logger(""), format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const char* format, Args&&... args) {
  RCLCPP_WARN(rclcpp::get_logger(""), format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(const char* format, Args&&... args) {
  RCLCPP_ERROR(rclcpp::get_logger(""), format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void fatal(const char* format, Args&&... args) {
  RCLCPP_FATAL(rclcpp::get_logger(""), format, std::forward<Args>(args)...);
}

}  // namespace fluorite

#endif  // COMMON_LOGGER_HPP