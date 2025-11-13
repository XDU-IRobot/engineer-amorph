
#include "arm_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmNode>();
  rclcpp::spin(node->ros_node());
  rclcpp::shutdown();
  return 0;
}