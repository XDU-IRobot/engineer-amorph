
#ifndef NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP
#define NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionCommandStorageSystem : public BT::SyncActionNode {
 public:
  static constexpr const char* name = "ActionCommandStorageSystem";

  ActionCommandStorageSystem(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP