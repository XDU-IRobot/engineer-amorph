
#ifndef NODES_ACTION_ARM_RETRACT_HPP
#define NODES_ACTION_ARM_RETRACT_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmRetract : public BT::SyncActionNode {
 public:
  ActionArmRetract(const std::string& name);

  BT::NodeStatus tick() override;

  static constexpr const char* name = "ActionArmBack";
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_ARM_RETRACT_HPP