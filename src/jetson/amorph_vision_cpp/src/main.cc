#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

#include "amorph_vision_cpp/eu_locator_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<amorph::vision::EuLocatorNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("eu_locator_node"), "LAUNCH FAIL: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
