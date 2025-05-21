
#ifndef NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP
#define NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include "fluorite_msgs/srv/storage_system_command.hpp"

#include "ros_node.hpp"

namespace bt_nodes {

class ActionCommandStorageSystem : public BT::SyncActionNode {
 public:
  static constexpr const char* name = "ActionCommandStorageSystem";

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("left"), BT::InputPort<std::string>("right")};
  }

  ActionCommandStorageSystem(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override {
    BT::Optional<std::string> left = getInput<std::string>("left");
    BT::Optional<std::string> right = getInput<std::string>("right");
    if (!left) {
      throw BT::RuntimeError("missing required input [left]: ", left.error());
    }
    if (!right) {
      throw BT::RuntimeError("missing required input [right]: ", right.error());
    }

    auto& ros_node = RosNodeSingleton::GetInstance();
    auto service_request = std::make_shared<fluorite_msgs::srv::StorageSystemCommand::Request>();

    if (left.value() == "on") {
      service_request->left = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_ON;
    } else if (left.value() == "off") {
      service_request->left = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_OFF;
    } else if (left.value() == "nope") {
      service_request->left = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_DONT_CHANGE;
    } else {
      throw BT::RuntimeError("invalid input [left]: ", left.value());
    }

    if (right.value() == "on") {
      service_request->right = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_ON;
    } else if (right.value() == "off") {
      service_request->right = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_OFF;
    } else if (right.value() == "nope") {
      service_request->right = fluorite_msgs::srv::StorageSystemCommand::Request::STORAGE_DONT_CHANGE;
    } else {
      throw BT::RuntimeError("invalid input [right]: ", right.value());
    }

    ros_node.storage_system_command_client_->async_send_request(
        service_request, [this](rclcpp::Client<fluorite_msgs::srv::StorageSystemCommand>::SharedFuture) {});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace bt_nodes

#endif  // NODES_ACTION_COMMAND_STORAGE_SYSTEM_HPP