
#ifndef NODES_ACTION_SET_NAVIGATION_TARGET_HPP
#define NODES_ACTION_SET_NAVIGATION_TARGET_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionSetNavigationTarget : public BT::SyncActionNode {
 public:
  static constexpr const char* name = "ActionSetNavigationTarget";

  ActionSetNavigationTarget(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_SET_NAVIGATION_TARGET_HPP