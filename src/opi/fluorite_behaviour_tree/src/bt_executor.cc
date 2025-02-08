
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "bt_node_register.hpp"

int main() {
  BT::BehaviorTreeFactory factory;
  bt_nodes::BtNodeRegisterArray::Register(factory);

  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("fluorite_behaviour_tree");
  const std::string tree_path = package_share_directory + "/xml/trees/pump_on_off_test.xml";

  std::cout << "Loading tree from file: " << tree_path << std::endl;

  auto tree = factory.createTreeFromFile(tree_path);
  tree.tickRootWhileRunning();
  return 0;
}