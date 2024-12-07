
#include <rclcpp/rclcpp.hpp>

#define debug(...) RCLCPP_DEBUG(rclcpp::get_logger("fluorite_chassis"), __VA_ARGS__)
#define info(...) RCLCPP_INFO(rclcpp::get_logger("fluorite_chassis"), __VA_ARGS__)
#define warn(...) RCLCPP_WARN(rclcpp::get_logger("fluorite_chassis"), __VA_ARGS__)
#define error(...) RCLCPP_ERROR(rclcpp::get_logger("fluorite_chassis"), __VA_ARGS__)