#include "amorph_custom_client/mtc/logger.hpp"

namespace amorph {

static rclcpp::Logger global_logger_instance = rclcpp::get_logger("amorph_custom_client");

rclcpp::Logger& L::Get() { return global_logger_instance; }

}  // namespace amorph
