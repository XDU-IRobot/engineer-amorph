
#include <rclcpp/rclcpp.hpp>

#include "amorph_transport/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amorph::TransportNode>();
  // node->InitRpc();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}