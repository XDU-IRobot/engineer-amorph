
#include "node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto amorph_node = std::make_shared<amorph::AmorphArmNode>();
  amorph_node->Spin();
  rclcpp::shutdown();
  return 0;
}
