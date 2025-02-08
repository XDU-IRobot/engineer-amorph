
#include "nodes/action_arm_retract.hpp"

namespace bt_nodes {

ActionArmRetract::ActionArmRetract(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ActionArmRetract::tick() { return BT::NodeStatus::SUCCESS; }

}  // namespace bt_nodes