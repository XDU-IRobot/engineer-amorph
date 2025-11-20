
#include "node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto amorph_node = std::make_shared<amorph::AmorphArmNode>();
  rclcpp::spin(amorph_node->ros_node());
  rclcpp::shutdown();
  return 0;
}
