#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Forward-declare Paho MQTT C++ types (full include only in .cc)
namespace mqtt {
class async_client;
class message;
}  // namespace mqtt

namespace amorph {

/**
 * @brief Encapsulates all MQTT broker interaction for the RoboMaster gateway.
 *
 * Owns:
 *   - Paho MQTT async_client lifecycle (connect / disconnect / reconnect)
 *   - /custom_control_cmd subscription → MQTT CustomControl publish
 *   - Poll timer to drive async delivery
 *
 * Connection is controlled via Connect() / Disconnect() (not automatic).
 * Intended to be driven by LifecycleNode transitions.
 *
 * Exposes minimal control & data interfaces:
 *   - Connect()        — connect to MQTT broker (auto-detects client_id)
 *   - Disconnect()     — tear down MQTT connection
 *   - SetOnByteBlock() — inject callback for incoming CustomByteBlock bytes
 *   - WriteControl()   — send bytes to robot via MQTT CustomControl
 *
 * Client ID auto-detection:
 *   Tries "102" first, then "2" if the server rejects the first one.
 *   No manual client_id parameter is needed.
 *
 * Parameters (read from owning node):
 *   - mqtt_host       (string, default "192.168.12.1")
 *   - mqtt_port       (int,    default 3333)
 *   - poll_period_ms  (int,    default 50)
 */
/// @tparam NodeT  Node-like type providing: declare_parameter, create_subscription,
///               create_wall_timer, get_logger, now, get_clock.
///               Typically rclcpp::Node or rclcpp_lifecycle::LifecycleNode.
template <typename NodeT>
class MqttClient {
 public:
  using ByteBlockCallback = std::function<void(const std::vector<uint8_t> &data)>;
  using ConnectionLostCallback = std::function<void()>;

  explicit MqttClient(NodeT &node);
  ~MqttClient();

  /// Connect to MQTT broker (auto-detect client_id 102/2).
  /// Returns true if connected successfully.
  bool Connect();

  /// Disconnect from MQTT broker and tear down async_client.
  void Disconnect();

  /// Inject handler for incoming robot → client messages (CustomByteBlock raw bytes).
  void SetOnByteBlock(ByteBlockCallback cb);

  /// Inject handler for connection-loss events (detected by Paho).
  /// The callback is invoked from Paho's internal thread — keep it fast.
  void SetOnConnectionLost(ConnectionLostCallback cb);

  /// Send raw bytes to robot via MQTT CustomControl topic.
  /// No-op (with throttled warning) when MQTT is not connected.
  void WriteControl(const std::vector<uint8_t> &data);

  /// True when the MQTT connection is established.
  bool IsConnected() const;

  /// The auto-detected client ID ("102" or "2"), empty before first connect.
  const std::string &GetClientId() const;

 private:
  // -- MQTT ---------------------------------------------------------------
  void TryConnect();
  void OnMqttMessage(const mqtt::message &msg);
  void OnMqttConnected();

  // -- ROS callbacks ------------------------------------------------------
  void OnRosControlCmd(const std::vector<uint8_t> &data);

  // -- Timer --------------------------------------------------------------
  void PollMqtt();

  NodeT &node_;

  // Parameters
  std::string mqtt_host_;
  int mqtt_port_;

  // MQTT
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::atomic<bool> connected_{false};
  std::string detected_client_id_;  // "102" or "2" after successful connect

  // ROS
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  // Callback slot
  ByteBlockCallback on_byte_block_;
  ConnectionLostCallback on_connection_lost_;
};

}  // namespace amorph