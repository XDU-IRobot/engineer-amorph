
#ifndef NODES_ACTION_COMMAND_ARM_JOINT_POSITION_HPP
#define NODES_ACTION_COMMAND_ARM_JOINT_POSITION_HPP

#include "behaviortree_cpp_v3/action_node.h"

class ActionCommandArmJointPosition : public BT::StatefulActionNode {
 public:
  static constexpr const char* name = "ActionCommandArmJointPosition";

  // If your Node has ports, you must use this constructor signature
  ActionCommandArmJointPosition(const std::string& name, const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config) {}

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts() {
    // This action has a single input port called "message"
    return {
        BT::InputPort<std::string>("srdf_pose_name"),
        BT::OutputPort<std::string>("trajectory"),
    };
  }

  // Override the virtual function tick()
  BT::NodeStatus onStart() override {
    BT::Optional<std::string> msg = getInput<std::string>("message");
    // Check if optional is valid. If not, throw its error
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

#endif