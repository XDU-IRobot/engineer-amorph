#include "amorph_client_gateway/mqtt_gateway_node.hpp"

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace amorph {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MqttGatewayNode::MqttGatewayNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("mqtt_gateway", options) {}

// ---------------------------------------------------------------------------
// Lifecycle: configure — create sub-objects (no MQTT connection yet)
// ---------------------------------------------------------------------------

MqttGatewayNode::CallbackReturn
MqttGatewayNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_configure() — creating MqttClient & CustomByteBlockHandler");

  mqtt_client_ = std::make_unique<MqttClient<rclcpp_lifecycle::LifecycleNode>>(*this);
  byte_block_handler_ = std::make_unique<CustomByteBlockHandler<rclcpp_lifecycle::LifecycleNode>>(*this);

  // Wire incoming CustomByteBlock → RobotFeedBack decoding
  mqtt_client_->SetOnByteBlock(
      [this](const std::vector<uint8_t>& data) {
        byte_block_handler_->ProcessRobotFeedBack(data);
      });

  // Wire MQTT connection loss → lifecycle deactivate
  mqtt_client_->SetOnConnectionLost([this]() {
    RCLCPP_WARN(get_logger(), "Connection lost — triggering lifecycle deactivate");
    trigger_transition(
        rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  });

  RCLCPP_INFO(get_logger(), "on_configure() — done");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: activate — connect to MQTT broker
// ---------------------------------------------------------------------------

MqttGatewayNode::CallbackReturn
MqttGatewayNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_activate() — connecting to MQTT broker");

  if (!mqtt_client_->Connect()) {
    RCLCPP_ERROR(get_logger(), "on_activate() — MQTT connect failed");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "on_activate() — connected (client_id=%s)",
              mqtt_client_->GetClientId().c_str());
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: deactivate — disconnect MQTT
// ---------------------------------------------------------------------------

MqttGatewayNode::CallbackReturn
MqttGatewayNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_deactivate() — disconnecting MQTT");

  if (mqtt_client_) {
    mqtt_client_->Disconnect();
  }

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: cleanup — destroy sub-objects
// ---------------------------------------------------------------------------

MqttGatewayNode::CallbackReturn
MqttGatewayNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_cleanup() — destroying sub-objects");

  if (mqtt_client_) {
    mqtt_client_->Disconnect();
    mqtt_client_.reset();
  }
  byte_block_handler_.reset();

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: shutdown
// ---------------------------------------------------------------------------

MqttGatewayNode::CallbackReturn
MqttGatewayNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "on_shutdown()");
  on_cleanup(state);
  return CallbackReturn::SUCCESS;
}

}  // namespace amorph

RCLCPP_COMPONENTS_REGISTER_NODE(amorph::MqttGatewayNode)
