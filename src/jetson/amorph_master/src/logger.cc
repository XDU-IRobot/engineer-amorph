
#include "logger.hpp"

namespace amorph {

static rclcpp::Logger global_logger_instance = rclcpp::get_logger("amorph_master");

rclcpp::Logger& L::Get() { return global_logger_instance; }

}  // namespace amorph