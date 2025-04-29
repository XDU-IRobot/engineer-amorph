
#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>

#define debug(...)                                     \
  do {                                                 \
    RCLCPP_DEBUG(rclcpp::get_logger(""), __VA_ARGS__); \
  } while (0);
#define info(...)                                     \
  do {                                                \
    RCLCPP_INFO(rclcpp::get_logger(""), __VA_ARGS__); \
  } while (0);
#define warn(...)                                     \
  do {                                                \
    RCLCPP_WARN(rclcpp::get_logger(""), __VA_ARGS__); \
  } while (0);
#define error(...)                                     \
  do {                                                 \
    RCLCPP_ERROR(rclcpp::get_logger(""), __VA_ARGS__); \
  } while (0);
#define fatal(...)                                     \
  do {                                                 \
    RCLCPP_FATAL(rclcpp::get_logger(""), __VA_ARGS__); \
  } while (0);

#endif  // COMMON_LOGGER_HPP