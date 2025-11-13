
#include "amorph_host/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmorphArmNode>();
  rclcpp::spin(node->ros_node());
  rclcpp::shutdown();
  return 0;
}