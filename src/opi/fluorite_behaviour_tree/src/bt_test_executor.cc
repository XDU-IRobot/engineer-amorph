
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "bt_node_register.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  bt_nodes::BtNodeRegisterArray::Register(factory);

  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("fluorite_behaviour_tree");
  const std::string tree_path = package_share_directory + "/xml/trees/go_to_right_storage_direct.xml";

  std::cout << "Loading tree from file: " << tree_path << std::endl;

  auto tree = factory.createTreeFromFile(tree_path);
  BT::StdCoutLogger logger_cout(tree);
  BT::PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();

  rclcpp::shutdown();
  return 0;
}