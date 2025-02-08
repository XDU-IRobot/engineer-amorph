
#include "nodes/action_arm_pump_on.hpp"

#include <iostream>

namespace bt_nodes {

ActionArmPumpOn::ActionArmPumpOn(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ActionArmPumpOn::tick() {
  std::cout << "Pump on" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes