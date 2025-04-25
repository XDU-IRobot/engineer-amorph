
#ifndef BT_NODE_REGISTER_HPP
#define BT_NODE_REGISTER_HPP

#include "behaviortree_cpp_v3/bt_factory.h"

#include "nodes/action_command_arm_end_effector_pose.hpp"
#include "nodes/action_command_arm_joint_position.hpp"
#include "nodes/action_command_arm_joint_trajectory.hpp"
#include "nodes/action_command_chassis.hpp"
#include "nodes/action_command_storage_system.hpp"
#include "nodes/action_command_sucker_system.hpp"
#include "nodes/action_plan_to_joint_position.hpp"
#include "nodes/action_plan_to_end_effector_pose.hpp"
#include "nodes/action_plan_to_lift_end_effector.hpp"

// NOTE: movegroup interface的setNamedTarget函数有一些奇怪的傻逼问题会segment fault，
// 不能直接用srdf模型里面定义好的姿态，所以这个action做不了
// #include "nodes/action_plan_to_srdf_pose.hpp"

namespace bt_nodes {

template <class... Node>
struct BtNodeRegister {
  inline static void Register(BT::BehaviorTreeFactory& factory) { (factory.registerNodeType<Node>(Node::name), ...); }
};

using BtNodeRegisterArray = BtNodeRegister<ActionCommandArmEndEffectorPose,  //
                                           ActionCommandArmJointPosition,    //
                                           ActionCommandArmJointTrajectory,  //
                                           ActionCommandChassis,             //
                                           ActionCommandStorageSystem,       //
                                           ActionCommandSuckerSystem,        //
                                           ActionPlanToJointPosition,        //
                                           ActionPlanToEndEffectorPose,      //
                                           ActionPlanToLiftEndEffector>;

}  // namespace bt_nodes

#endif  // BT_NODE_REGISTER_HPP