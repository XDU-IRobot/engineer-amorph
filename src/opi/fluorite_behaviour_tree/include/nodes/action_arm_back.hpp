
#ifndef NODES_ACTION_ARM_BACK_HPP
#define NODES_ACTION_ARM_BACK_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmBack : public BT::SyncActionNode {
 public:
  ActionArmBack(const std::string& name);

  BT::NodeStatus tick() override;

  static constexpr const char* name = "ActionArmBack";
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_ARM_BACK_HPP