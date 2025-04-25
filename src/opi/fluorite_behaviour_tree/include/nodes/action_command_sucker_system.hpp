
#ifndef NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP
#define NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionCommandSuckerSystem : public BT::SyncActionNode {
 public:
  static constexpr const char* name = "ActionCommandSuckerSystem";

  ActionCommandSuckerSystem(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP