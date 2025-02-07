
#include "behaviortree_cpp_v3/bt_factory.h"

BT::BehaviorTreeFactory global_bt_factory;

BT::BehaviorTreeFactory& GetBtFactoryInstance() { return global_bt_factory; }
