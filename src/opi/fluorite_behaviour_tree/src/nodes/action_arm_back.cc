
#include "behaviortree_cpp_v3/action_node.h"

namespace bt_nodes {

class ActionArmBack : public BT::SyncActionNode {
 public:
  ActionArmBack(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_nodes