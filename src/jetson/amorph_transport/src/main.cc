
#include <rclcpp/rclcpp.hpp>

#include "amorph_transport/arm_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amorph::ArmControllerNode>();
  node->InitRpc();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}