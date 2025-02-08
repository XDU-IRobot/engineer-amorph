
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nodes/action_arm_retract.hpp"
#include "nodes/action_arm_extend_above_silver.hpp"
#include "nodes/action_arm_pump_off.hpp"
#include "nodes/action_arm_pump_on.hpp"

namespace bt_nodes {

template <class... Node>
struct BtNodeRegister {
  inline static void Register(BT::BehaviorTreeFactory& factory) { (factory.registerNodeType<Node>(Node::name), ...); }
};

using BtNodeRegisterArray =
    BtNodeRegister<ActionArmRetract, ActionArmExtendAboveSilver, ActionArmPumpOff, ActionArmPumpOn>;

}  // namespace bt_nodes