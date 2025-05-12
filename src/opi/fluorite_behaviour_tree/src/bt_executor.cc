
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "bt_node_register.hpp"
#include "tree_loader.hpp"

#include "moveit.hpp"

class BTExecutorNode : public rclcpp::Node {
 public:
  explicit BTExecutorNode(
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : Node("bt_executor_node", options) {
    bt_nodes::BtNodeRegisterArray::Register(factory_);
  }

 private:
  BT::BehaviorTreeFactory factory_;
  TreeLoader tree_loader_{factory_};
};

int main(int argc, char** argv) try {
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
} catch (const std::exception& e) {
  RCLCPP_ERROR(rclcpp::get_logger("fluorite"), "Exception: %s", e.what());
  rclcpp::shutdown();
  return -1;
} catch (...) {
  RCLCPP_ERROR(rclcpp::get_logger("fluorite"), "Unknown exception");
  rclcpp::shutdown();
  return -1;
}