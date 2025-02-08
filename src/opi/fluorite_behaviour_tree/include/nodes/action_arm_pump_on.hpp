
#ifndef NODES_ACTION_ARM_PUMP_ON_HPP
#define NODES_ACTION_ARM_PUMP_ON_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmPumpOn : public BT::SyncActionNode {
 public:
  ActionArmPumpOn(const std::string& name);

  BT::NodeStatus tick() override;

  static constexpr const char* name = "ActionArmPumpOn";
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_ARM_PUMP_ON_HPP