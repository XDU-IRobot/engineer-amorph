
#ifndef NODES_ACTION_ARM_EXTEND_ABOVE_SILVER
#define NODES_ACTION_ARM_EXTEND_ABOVE_SILVER

#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmExtendAboveSilver : public BT::SyncActionNode {
 public:
  ActionArmExtendAboveSilver(const std::string& name);

  BT::NodeStatus tick() override;

  static constexpr const char* name = "ActionArmExtendAboveSilver";
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_ARM_EXTEND_ABOVE_SILVER