
#ifndef NODES_ACTION_ARM_PUMP_OFF_HPP
#define NODES_ACTION_ARM_PUMP_OFF_HPP

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmPumpOff : public BT::SyncActionNode {
 public:
  ActionArmPumpOff(const std::string& name);

  BT::NodeStatus tick() override;

  static constexpr const char* name = "ActionArmPumpOff";
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_ARM_PUMP_OFF_HPP