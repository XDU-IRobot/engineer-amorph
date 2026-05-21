#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

#include "amorph_client_gateway/custom_byte_block_handler.hpp"
#include "amorph_client_gateway/mqtt_client.hpp"

namespace amorph {

/**
 * @brief Lifecycle MQTT Gateway Node — bridges RoboMaster custom client MQTT ↔ ROS 2.
 *
 * Lifecycle state machine:
 *   unconfigured → on_configure()  → creates MqttClient + CustomByteBlockHandler
 *                                    (no MQTT connection yet)
 *   inactive     → on_activate()   → connects to MQTT broker
 *   active       → on_deactivate() → disconnects MQTT
 *   inactive     → on_cleanup()    → destroys sub-objects
 *
 * Parameters (consumed by sub-objects):
 *   - MqttClient:          mqtt_host, mqtt_port, poll_period_ms
 *   - CustomByteBlockHandler: joint_names, camera_frame
 *
 * Publishes (via CustomByteBlockHandler):
 *   - /joint_states (sensor_msgs/JointState)
 *   - /amorph/system_status (amorph_interfaces/SystemStatus)
 *   - <camera_frame> → energy_unit_0..7 TF
 */
class MqttGatewayNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MqttGatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    std::unique_ptr<MqttClient<rclcpp_lifecycle::LifecycleNode>> mqtt_client_;
    std::unique_ptr<CustomByteBlockHandler<rclcpp_lifecycle::LifecycleNode>> byte_block_handler_;
};

}  // namespace amorph
