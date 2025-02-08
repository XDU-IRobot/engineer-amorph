
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nodes/action_arm_back.hpp"
#include "nodes/action_arm_extend_above_silver.hpp"

namespace bt_nodes {

template <class... Node>
struct BtNodeRegister {
  inline static void Register(BT::BehaviorTreeFactory& factory) { (factory.registerNodeType<Node>(Node::name), ...); }
};

using BtNodeRegisterArray = BtNodeRegister<ActionArmBack, ActionArmExtendAboveSilver>;

}  // namespace bt_nodes