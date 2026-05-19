#include "amorph_client_gateway/mqtt_gateway_node.hpp"

#include <mqtt/async_client.h>
#include <mqtt/connect_options.h>
#include <mqtt/message.h>

#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "custom_byte_block.pb.h"
#include "custom_control.pb.h"

namespace amorph {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MqttGatewayNode::MqttGatewayNode(const rclcpp::NodeOptions& options)
    : Node("mqtt_gateway", options) {
    // -- parameters ----------------------------------------------------------
    _mqtt_host = this->declare_parameter("mqtt_host", std::string("192.168.12.1"));
    _mqtt_port = this->declare_parameter("mqtt_port", 3333);
    _client_id = this->declare_parameter("client_id", std::string(""));
    int poll_period_ms = this->declare_parameter("poll_period_ms", 50);

    // -- ROS publishers / subscribers ----------------------------------------
    _custom_byte_block_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "custom_byte_block", rclcpp::QoS(10).reliable());

    _custom_control_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "custom_control_cmd", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
            on_ros_control_cmd(msg);
        });

    // -- MQTT client ---------------------------------------------------------
    connect_mqtt();

    // -- Polling timer (drives async MQTT message delivery) ------------------
    _poll_timer = this->create_wall_timer(
        std::chrono::milliseconds(poll_period_ms),
        [this]() { poll_mqtt(); });

    RCLCPP_INFO(this->get_logger(),
        "MQTT Gateway started: %s:%d (client_id=%s)",
        _mqtt_host.c_str(), _mqtt_port,
        _client_id.empty() ? "(auto)" : _client_id.c_str());
}

MqttGatewayNode::~MqttGatewayNode() {
    _poll_timer.reset();
    if (_mqtt_client && _mqtt_client->is_connected()) {
        try {
            _mqtt_client->disconnect()->wait();
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "MQTT disconnect error: %s", e.what());
        }
    }
    _connected = false;
}

// ---------------------------------------------------------------------------
// Public API — callback injection
// ---------------------------------------------------------------------------

void MqttGatewayNode::set_on_custom_byte_block(ByteBlockCallback cb) {
    _on_byte_block = std::move(cb);
}

void MqttGatewayNode::set_on_connection_change(StatusCallback cb) {
    _on_connection_change = std::move(cb);
}

// ---------------------------------------------------------------------------
// Private: MQTT
// ---------------------------------------------------------------------------

void MqttGatewayNode::connect_mqtt() {
    try {
        std::string server_uri = "mqtt://" + _mqtt_host + ":" + std::to_string(_mqtt_port);

        _mqtt_client = std::make_unique<mqtt::async_client>(server_uri, _client_id);

        // Register callbacks (Paho will invoke these from its internal thread)
        _mqtt_client->set_message_callback(
            [this](mqtt::const_message_ptr msg) {
                if (msg) on_mqtt_message(*msg);
            });

        _mqtt_client->set_connected_handler(
            [this](const std::string& /*cause*/) { on_mqtt_connected(); });

        _mqtt_client->set_connection_lost_handler(
            [this](const std::string& cause) {
                RCLCPP_WARN(this->get_logger(), "MQTT connection lost: %s", cause.c_str());
                _connected = false;
                if (_on_connection_change) _on_connection_change(false);
            });

        // Connect
        mqtt::connect_options conn_opts;
        conn_opts.set_clean_session(true);
        conn_opts.set_automatic_reconnect(true);
        conn_opts.set_keep_alive_interval(10);

        RCLCPP_INFO(this->get_logger(), "Connecting to MQTT broker %s ...", server_uri.c_str());
        _mqtt_client->connect(conn_opts)->wait();

        // Subscribe to robot → client topic
        _mqtt_client->subscribe("CustomByteBlock", 1)->wait();
        RCLCPP_INFO(this->get_logger(), "Subscribed to CustomByteBlock");

    } catch (const mqtt::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MQTT connect failed: %s (rc=%d)", e.what(),
                     e.get_reason_code());
        // Destroy client to prevent zombie reconnect threads.
        // Paho's automatic_reconnect thread-creation can fail with
        // std::system_error (EPERM) on systems with thread limits.
        _mqtt_client.reset();
        RCLCPP_WARN(this->get_logger(),
            "MQTT is not available. Node will stay alive and log any "
            "outgoing CustomControl drops. "
            "Make sure the robot is connected and 192.168.12.1:3333 is reachable.");
    } catch (const std::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "MQTT thread creation failed: %s", e.what());
        _mqtt_client.reset();
        RCLCPP_WARN(this->get_logger(),
            "System thread limit may be too low. Node will operate without MQTT.");
    }
}

void MqttGatewayNode::on_mqtt_connected() {
    _connected = true;
    RCLCPP_INFO(this->get_logger(), "MQTT connected to %s:%d", _mqtt_host.c_str(), _mqtt_port);
    if (_on_connection_change) _on_connection_change(true);
}

void MqttGatewayNode::on_mqtt_disconnected() {
    _connected = false;
    if (_on_connection_change) _on_connection_change(false);
}

void MqttGatewayNode::on_mqtt_message(const mqtt::message& msg) {
    const auto& payload = msg.get_payload_str();

    if (msg.get_topic() == "CustomByteBlock") {
        // Deserialize Protobuf
        amorph::proto::CustomByteBlock block;
        if (!block.ParseFromString(payload)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to parse CustomByteBlock protobuf");
            return;
        }

        const auto& data = block.data();

        // Publish raw bytes to ROS
        std_msgs::msg::UInt8MultiArray ros_msg;
        ros_msg.data.assign(data.begin(), data.end());
        _custom_byte_block_pub->publish(ros_msg);

        // Invoke injected handler (if any)
        if (_on_byte_block) {
            _on_byte_block(std::vector<uint8_t>(data.begin(), data.end()));
        }
    }
}

// ---------------------------------------------------------------------------
// Private: ROS → MQTT
// ---------------------------------------------------------------------------

void MqttGatewayNode::on_ros_control_cmd(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (!_mqtt_client || !_connected) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "MQTT not connected, dropping CustomControl message");
        return;
    }

    // Serialize to Protobuf
    amorph::proto::CustomControl ctrl;
    ctrl.set_data(msg->data.data(), msg->data.size());

    std::string serialized;
    if (!ctrl.SerializeToString(&serialized)) {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize CustomControl");
        return;
    }

    try {
        auto mqtt_msg = mqtt::make_message("CustomControl", serialized);
        mqtt_msg->set_qos(1);
        _mqtt_client->publish(mqtt_msg);
    } catch (const mqtt::exception& e) {
        RCLCPP_WARN(this->get_logger(), "MQTT publish failed: %s", e.what());
    }
}

// ---------------------------------------------------------------------------
// Private: poll loop
// ---------------------------------------------------------------------------

void MqttGatewayNode::poll_mqtt() {
    // Paho async_client doesn't strictly require polling, but calling
    // consume_message helps drive the callback dispatch when using
    // certain Paho versions.  No-op if no message pending.
    if (_mqtt_client) {
        try {
            mqtt::const_message_ptr msg;
            _mqtt_client->try_consume_message(&msg);
        } catch (const mqtt::exception&) {
            // Ignore — delivery is callback-driven
        }
    }
}

}  // namespace amorph

RCLCPP_COMPONENTS_REGISTER_NODE(amorph::MqttGatewayNode)
