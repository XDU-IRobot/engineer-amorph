
#ifndef NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP
#define NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP

#include <behaviortree_cpp_v3/basic_types.h>
#include "behaviortree_cpp_v3/action_node.h"

#include "ros_node.hpp"

namespace bt_nodes {

class ActionCommandSuckerSystem : public BT::SyncActionNode {
 public:
  static constexpr const char* name = "ActionCommandSuckerSystem";

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("command")}; }

  ActionCommandSuckerSystem(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    BT::Optional<std::string> command = getInput<std::string>("command");
    if (!command) {
      throw BT::RuntimeError("missing required input [command]: ", command.error());
    }
    auto& ros_node = RosNodeSingleton::GetInstance();
    auto service_request = std::make_shared<fluorite_msgs::srv::SuckerCommand::Request>();

    if (command.value() == "suck") {
      service_request->command = fluorite_msgs::srv::SuckerCommand::Request::SUCKER_SUCK;
    } else if (command.value() == "blow") {
      service_request->command = fluorite_msgs::srv::SuckerCommand::Request::SUCKER_BLOW;
    } else if (command.value() == "off") {
      service_request->command = fluorite_msgs::srv::SuckerCommand::Request::SUCKER_OFF;
    } else {
      throw BT::RuntimeError("invalid input [command]: ", command.value());
    }

    auto result_future = ros_node.sucker_command_client_->async_send_request(
        service_request, [this](rclcpp::Client<fluorite_msgs::srv::SuckerCommand>::SharedFuture) {});
    result_future.wait();

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_COMMAND_SUCKER_SYSTEM_HPP