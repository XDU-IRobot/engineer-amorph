
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "bt_node_register.hpp"
#include "tree_loader.hpp"

#include "moveit.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  bt_nodes::BtNodeRegisterArray::Register(factory);
  TreeLoader tree_loader(factory);

  auto tree = tree_loader.Load("take_silver_and_store");
  BT::StdCoutLogger logger_cout(tree);
  BT::PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();

  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   // MoveGroupInterfaceSingleton::GetInstance().GetMoveGroupInterface();

//   rclcpp::shutdown();
//   return 0;
// }