#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <amorph_interfaces/msg/system_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

namespace amorph {

/// @brief Handles CustomByteBlock → RobotFeedBack decoding and publishes:
///        - /joint_states (sensor_msgs/JointState)
///        - /amorph/system_status (amorph_interfaces/SystemStatus)
///        - <camera_frame> → energy_unit_{0..7} TF (only when detected)
///
/// All ROS resources (publishers, TF broadcaster) are owned by this class.
/// Parameters are read from the owning node.
/// @tparam NodeT  Node-like type providing: declare_parameter, create_publisher,
///               get_logger, now, get_clock, *get_clock().
///               Typically rclcpp::Node or rclcpp_lifecycle::LifecycleNode.
template <typename NodeT>
class CustomByteBlockHandler {
 public:
  explicit CustomByteBlockHandler(NodeT &node);

  /// Parse RobotFeedBack from raw bytes → publish JointState / SystemStatus / TF.
  void ProcessRobotFeedBack(const std::vector<uint8_t> &data);

 private:
  NodeT &node_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::string camera_frame_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<amorph_interfaces::msg::SystemStatus>::SharedPtr system_status_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amorph