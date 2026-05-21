#include "amorph_rviz_plugins/mtc/logger.hpp"

namespace amorph {

static rclcpp::Logger global_logger_instance = rclcpp::get_logger("amorph_rviz_plugins");

rclcpp::Logger& L::Get() { return global_logger_instance; }

}  // namespace amorph
