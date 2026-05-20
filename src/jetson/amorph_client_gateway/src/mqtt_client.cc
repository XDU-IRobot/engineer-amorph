#include "amorph_client_gateway/mqtt_client.hpp"

#include <mqtt/async_client.h>
#include <mqtt/connect_options.h>
#include <mqtt/message.h>

#include <array>
#include <chrono>
#include <cstring>

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "custom_byte_block.pb.h"
#include "custom_control.pb.h"

namespace amorph {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

template <typename NodeT>
MqttClient<NodeT>::MqttClient(NodeT &node) : node_(node) {
  mqtt_host_ = node_.declare_parameter("mqtt_host", std::string("192.168.12.1"));
  mqtt_port_ = node_.declare_parameter("mqtt_port", 3333);
  int poll_period_ms = node_.declare_parameter("poll_period_ms", 50);

  // -- Subscription: /custom_control_cmd → MQTT CustomControl ------------
  control_sub_ = node_.template create_subscription<std_msgs::msg::UInt8MultiArray>(
      "custom_control_cmd", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) { OnRosControlCmd(msg->data); });

  // -- Poll timer ---------------------------------------------------------
  poll_timer_ = node_.create_wall_timer(std::chrono::milliseconds(poll_period_ms), [this]() { PollMqtt(); });

  RCLCPP_INFO(node_.get_logger(), "MqttClient created: %s:%d", mqtt_host_.c_str(), mqtt_port_);
}

template <typename NodeT>
MqttClient<NodeT>::~MqttClient() {
  Disconnect();
}

// ---------------------------------------------------------------------------
// Public API — lifecycle
// ---------------------------------------------------------------------------

template <typename NodeT>
bool MqttClient<NodeT>::Connect() {
  TryConnect();
  return (mqtt_client_ != nullptr && connected_);
}

template <typename NodeT>
void MqttClient<NodeT>::Disconnect() {
  poll_timer_->cancel();
  if (mqtt_client_ && mqtt_client_->is_connected()) {
    try {
      mqtt_client_->disconnect()->wait();
    } catch (const std::exception &e) {
      RCLCPP_WARN(node_.get_logger(), "MQTT disconnect error: %s", e.what());
    }
  }
  mqtt_client_.reset();
  connected_ = false;
  detected_client_id_.clear();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

template <typename NodeT>
void MqttClient<NodeT>::SetOnByteBlock(ByteBlockCallback cb) { on_byte_block_ = std::move(cb); }

template <typename NodeT>
void MqttClient<NodeT>::SetOnConnectionLost(ConnectionLostCallback cb) { on_connection_lost_ = std::move(cb); }

template <typename NodeT>
void MqttClient<NodeT>::WriteControl(const std::vector<uint8_t> &data) {
  if (!mqtt_client_ || !connected_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "MQTT not connected, dropping CustomControl message");
    return;
  }

  // Serialize to Protobuf
  amorph::proto::CustomControl ctrl;
  ctrl.set_data(data.data(), data.size());

  std::string serialized;
  if (!ctrl.SerializeToString(&serialized)) {
    RCLCPP_WARN(node_.get_logger(), "Failed to serialize CustomControl");
    return;
  }

  try {
    auto mqtt_msg = mqtt::make_message("CustomControl", serialized);
    mqtt_msg->set_qos(1);
    mqtt_client_->publish(mqtt_msg);
  } catch (const mqtt::exception &e) {
    RCLCPP_WARN(node_.get_logger(), "MQTT publish failed: %s", e.what());
  }
}

template <typename NodeT>
bool MqttClient<NodeT>::IsConnected() const { return connected_; }

template <typename NodeT>
const std::string &MqttClient<NodeT>::GetClientId() const { return detected_client_id_; }

// ---------------------------------------------------------------------------
// Private: MQTT connection
// ---------------------------------------------------------------------------

template <typename NodeT>
void MqttClient<NodeT>::TryConnect() {
  // Server only accepts "102" or "2" — try each in order.
  static const std::array<std::string, 2> kCandidateIds = {"102", "2"};

  for (const auto &id : kCandidateIds) {
    try {
      std::string server_uri = "mqtt://" + mqtt_host_ + ":" + std::to_string(mqtt_port_);

      mqtt_client_ = std::make_unique<mqtt::async_client>(server_uri, id);

      // Register callbacks (Paho will invoke these from its internal thread)
      mqtt_client_->set_message_callback([this](mqtt::const_message_ptr msg) {
        if (msg) OnMqttMessage(*msg);
      });

      mqtt_client_->set_connected_handler([this](const std::string & /*cause*/) { OnMqttConnected(); });

      mqtt_client_->set_connection_lost_handler([this](const std::string &cause) {
        RCLCPP_WARN(node_.get_logger(), "MQTT connection lost: %s", cause.c_str());
        connected_ = false;
        if (on_connection_lost_) {
          on_connection_lost_();
        }
      });

      // Connect
      mqtt::connect_options conn_opts;
      conn_opts.set_clean_session(true);
      conn_opts.set_automatic_reconnect(true);
      conn_opts.set_keep_alive_interval(10);

      RCLCPP_INFO(node_.get_logger(), "Connecting to MQTT broker %s with client_id=%s ...", server_uri.c_str(),
                  id.c_str());
      mqtt_client_->connect(conn_opts)->wait();

      // Subscribe to robot → client topic
      mqtt_client_->subscribe("CustomByteBlock", 1)->wait();

      detected_client_id_ = id;
      RCLCPP_INFO(node_.get_logger(), "Subscribed to CustomByteBlock (client_id=%s)", id.c_str());
      return;  // success

    } catch (const mqtt::exception &e) {
      const char *what = e.what();
      // CONNACK errors mean the server rejected us at the MQTT protocol
      // level (e.g. bad client_id) — worth trying the next candidate.
      // Network-level errors (timeout, refused, host unreachable) won't
      // contain "CONNACK" — switching ID won't help, stop early.
      bool is_connack = (std::strstr(what, "CONNACK") != nullptr);

      RCLCPP_WARN(node_.get_logger(), "MQTT connect with client_id=%s %s: %s (rc=%d)", id.c_str(),
                  is_connack ? "rejected by server" : "failed", what, e.get_reason_code());
      mqtt_client_.reset();

      if (!is_connack) {
        // Network error — no point trying more IDs, the broker is unreachable
        RCLCPP_ERROR(node_.get_logger(),
                     "MQTT broker %s:%d is not reachable. "
                     "Make sure the robot is connected.",
                     mqtt_host_.c_str(), mqtt_port_);
        return;
      }
      // CONNACK rejection → try next candidate
    } catch (const std::system_error &e) {
      RCLCPP_ERROR(node_.get_logger(), "MQTT thread creation failed with client_id=%s: %s", id.c_str(), e.what());
      mqtt_client_.reset();
      // Thread creation failure is a system-level problem — stop early
      return;
    }
  }

  // All candidates failed
  RCLCPP_ERROR(node_.get_logger(),
               "MQTT connect exhausted all client_id candidates. "
               "Node will stay alive and log any outgoing CustomControl drops. "
               "Make sure the robot is connected and %s:%d is reachable.",
               mqtt_host_.c_str(), mqtt_port_);
}

template <typename NodeT>
void MqttClient<NodeT>::OnMqttConnected() {
  connected_ = true;
  RCLCPP_INFO(node_.get_logger(), "MQTT connected to %s:%d", mqtt_host_.c_str(), mqtt_port_);
}

template <typename NodeT>
void MqttClient<NodeT>::OnMqttMessage(const mqtt::message &msg) {
  const auto &payload = msg.get_payload_str();

  if (msg.get_topic() == "CustomByteBlock") {
    // Deserialize Protobuf
    amorph::proto::CustomByteBlock block;
    if (!block.ParseFromString(payload)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "Failed to parse CustomByteBlock protobuf");
      return;
    }

    // Forward raw bytes to injected handler
    const auto &data = block.data();
    if (on_byte_block_) {
      on_byte_block_(std::vector<uint8_t>(data.begin(), data.end()));
    }
  }
}

// ---------------------------------------------------------------------------
// Private: ROS subscription → MQTT
// ---------------------------------------------------------------------------

template <typename NodeT>
void MqttClient<NodeT>::OnRosControlCmd(const std::vector<uint8_t> &data) { WriteControl(data); }

// ---------------------------------------------------------------------------
// Private: poll loop
// ---------------------------------------------------------------------------

template <typename NodeT>
void MqttClient<NodeT>::PollMqtt() {
  if (mqtt_client_) {
    try {
      mqtt::const_message_ptr msg;
      mqtt_client_->try_consume_message(&msg);
    } catch (const mqtt::exception &) {
      // Ignore — delivery is callback-driven
    }
  }
}

// Explicit template instantiations
template class MqttClient<rclcpp::Node>;
template class MqttClient<rclcpp_lifecycle::LifecycleNode>;

}  // namespace amorph
