
#include "nodes/action_arm_extend_above_silver.hpp"

namespace bt_nodes {

ActionArmExtendAboveSilver::ActionArmExtendAboveSilver(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ActionArmExtendAboveSilver::tick() { return BT::NodeStatus::SUCCESS; }

}  // namespace bt_nodes