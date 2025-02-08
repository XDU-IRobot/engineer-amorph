
#include "nodes/action_arm_pump_off.hpp"

#include <iostream>

namespace bt_nodes {

ActionArmPumpOff::ActionArmPumpOff(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ActionArmPumpOff::tick() {
  std::cout << "Pump off" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes