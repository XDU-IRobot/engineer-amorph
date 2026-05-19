#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Forward-declare Paho MQTT C++ types (full include only in .cc)
namespace mqtt {
class async_client;
class connect_options;
class message;
}

namespace amorph {

/**
 * @brief MQTT Gateway Node — bridges RoboMaster custom client MQTT ↔ ROS 2.
 *
 * Architecture (per RoboMaster 2026 通信协议 §2):
 *
 *   Robot ──CustomByteBlock(50Hz)──→ MQTT Broker ──→ this node ──→ /custom_byte_block
 *   Robot ←── CustomControl(75Hz) ── MQTT Broker ←── this node ←── /custom_control_cmd
 *
 * MQTT Broker: 192.168.12.1:3333 (裁判系统服务器)
 *
 * Parameters:
 *   - mqtt_host     (string, default "192.168.12.1")
 *   - mqtt_port     (int,    default 3333)
 *   - client_id     (string, default "")   — used as MQTT client ID, set to robot ID
 *   - poll_period_ms (int,   default 50)   — how often to poll MQTT messages
 *
 * Publishers:
 *   - /custom_byte_block (std_msgs/UInt8MultiArray) — raw bytes from robot
 *
 * Subscriptions:
 *   - /custom_control_cmd (std_msgs/UInt8MultiArray) — commands to send to robot
 *
 * @note After construction, call set_on_custom_byte_block() to inject
 *       application-level message handling logic.
 */
class MqttGatewayNode : public rclcpp::Node {
public:
    /// Callback signature for received CustomByteBlock payloads.
    using ByteBlockCallback = std::function<void(const std::vector<uint8_t>& data)>;

    /// Callback signature for connection status changes.
    using StatusCallback = std::function<void(bool connected)>;

    explicit MqttGatewayNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true));

    ~MqttGatewayNode() override;

    /// Inject handler for incoming robot → client messages (CustomByteBlock topic).
    void set_on_custom_byte_block(ByteBlockCallback cb);

    /// Inject handler for connection state changes.
    void set_on_connection_change(StatusCallback cb);

private:
    // -- MQTT callbacks (called from Paho's internal thread) -----------------
    void on_mqtt_message(const mqtt::message& msg);
    void on_mqtt_connected();
    void on_mqtt_disconnected();

    // -- ROS callbacks -------------------------------------------------------
    void on_ros_control_cmd(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    // -- Timer ---------------------------------------------------------------
    /// Poll MQTT client for incoming messages (drives async delivery).
    void poll_mqtt();

    // -- Internal helpers ----------------------------------------------------
    void connect_mqtt();

    // Parameters
    std::string _mqtt_host;
    int _mqtt_port;
    std::string _client_id;

    // MQTT
    std::unique_ptr<mqtt::async_client> _mqtt_client;

    // ROS publishers & subscribers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _custom_byte_block_pub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _custom_control_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr _poll_timer;

    // Callback slots
    ByteBlockCallback _on_byte_block;
    StatusCallback _on_connection_change;

    // State
    std::atomic<bool> _connected{false};
};

}  // namespace amorph
