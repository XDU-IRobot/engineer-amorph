
#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>

#define debug(...) RCLCPP_DEBUG(rclcpp::get_logger("fluorite_control"), __VA_ARGS__)
#define info(...) RCLCPP_INFO(rclcpp::get_logger("fluorite_control"), __VA_ARGS__)
#define warn(...) RCLCPP_WARN(rclcpp::get_logger("fluorite_control"), __VA_ARGS__)
#define error(...) RCLCPP_ERROR(rclcpp::get_logger("fluorite_control"), __VA_ARGS__)

#endif  // COMMON_LOGGER_HPP