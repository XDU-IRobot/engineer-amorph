
#include "nodes/action_arm_back.hpp"

namespace bt_nodes {

ActionArmBack::ActionArmBack(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ActionArmBack::tick() { return BT::NodeStatus::SUCCESS; }

}  // namespace bt_nodes