#include "amorph_client_gateway/custom_byte_block_handler.hpp"

#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amorph_client_gateway/schema.hpp"

namespace amorph {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

template <typename NodeT>
CustomByteBlockHandler<NodeT>::CustomByteBlockHandler(NodeT &node) : node_(node) {
  joint_names_ = node_.declare_parameter("joint_names", std::vector<std::string>{});
  camera_frame_ = node_.declare_parameter("camera_frame", std::string("vt_camera_frame"));

  joint_state_pub_ =
      node_.template create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).reliable());

  system_status_pub_ =
      node_.template create_publisher<amorph_interfaces::msg::SystemStatus>("amorph/system_status", rclcpp::QoS(10).reliable());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

// ---------------------------------------------------------------------------
// RobotFeedBack parsing
// ---------------------------------------------------------------------------

template <typename NodeT>
void CustomByteBlockHandler<NodeT>::ProcessRobotFeedBack(const std::vector<uint8_t> &data) {
  if (data.size() < sizeof(schema::RobotFeedBack)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "CustomByteBlock too small: %zu < %zu", data.size(), sizeof(schema::RobotFeedBack));
    return;
  }

  schema::RobotFeedBack fb;
  std::memcpy(&fb, data.data(), sizeof(fb));

  auto now = node_.now();

  // -- 1. JointState -------------------------------------------------------
  if (!joint_names_.empty()) {
    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now;
    js_msg.name = joint_names_;
    js_msg.position.reserve(joint_names_.size());

    // Left arm: 6 joints
    for (int i = 0; i < 6; ++i) {
      js_msg.position.push_back(fb.js.left[i]);
    }
    // Left gripper (bool → position)
    js_msg.position.push_back(fb.js.left_gripper_open ? 0.0 : -0.02);

    // Right arm: 6 joints
    for (int i = 0; i < 6; ++i) {
      js_msg.position.push_back(fb.js.right[i]);
    }
    // Right gripper (bool → position)
    js_msg.position.push_back(fb.js.right_gripper_open ? 0.0 : -0.02);

    joint_state_pub_->publish(js_msg);
  }

  // -- 2. SystemStatus -----------------------------------------------------
  {
    amorph_interfaces::msg::SystemStatus ss_msg;
    ss_msg.header.stamp = now;

    // Joint / actuator health bits
    ss_msg.left_j1 = fb.ss.left_j1;
    ss_msg.left_j2 = fb.ss.left_j2;
    ss_msg.left_j3 = fb.ss.left_j3;
    ss_msg.left_j4 = fb.ss.left_j4;
    ss_msg.left_j5 = fb.ss.left_j5;
    ss_msg.left_j6 = fb.ss.left_j6;
    ss_msg.left_gripper = fb.ss.left_gripper;
    ss_msg.right_j1 = fb.ss.right_j1;
    ss_msg.right_j2 = fb.ss.right_j2;
    ss_msg.right_j3 = fb.ss.right_j3;
    ss_msg.right_j4 = fb.ss.right_j4;
    ss_msg.right_j5 = fb.ss.right_j5;
    ss_msg.right_j6 = fb.ss.right_j6;
    ss_msg.right_gripper = fb.ss.right_gripper;
    ss_msg.front_steer = fb.ss.front_steer;
    ss_msg.front_wheel = fb.ss.front_wheel;
    ss_msg.left_steer = fb.ss.left_steer;
    ss_msg.left_wheel = fb.ss.left_wheel;
    ss_msg.right_steer = fb.ss.right_steer;
    ss_msg.right_wheel = fb.ss.right_wheel;
    ss_msg.gimbal_yaw = fb.ss.gimbal_yaw;
    ss_msg.camera_lift = fb.ss.camera_lift;

    // Temperatures
    ss_msg.left_j3_temp = fb.ss.left_j3_temp;
    ss_msg.left_gripper_temp = fb.ss.left_gripper_temp;
    ss_msg.right_j3_temp = fb.ss.right_j3_temp;
    ss_msg.right_gripper_temp = fb.ss.right_gripper_temp;

    // FSM state
    ss_msg.fsm_state = fb.ss.fsm_state;

    system_status_pub_->publish(ss_msg);
  }

  // -- 3. EnergyUnitDetection TF ------------------------------------------
  for (int i = 0; i < 8; ++i) {
    if (!fb.eud[i].detected) {
      continue;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = camera_frame_;
    tf.child_frame_id = "energy_unit_" + std::to_string(i);

    tf.transform.translation.x = fb.eud[i].p_xyz[0];
    tf.transform.translation.y = fb.eud[i].p_xyz[1];
    tf.transform.translation.z = fb.eud[i].p_xyz[2];

    tf.transform.rotation.x = fb.eud[i].q_xyzw[0];
    tf.transform.rotation.y = fb.eud[i].q_xyzw[1];
    tf.transform.rotation.z = fb.eud[i].q_xyzw[2];
    tf.transform.rotation.w = fb.eud[i].q_xyzw[3];

    tf_broadcaster_->sendTransform(tf);
  }
}

// Explicit template instantiations
template class CustomByteBlockHandler<rclcpp::Node>;
template class CustomByteBlockHandler<rclcpp_lifecycle::LifecycleNode>;

}  // namespace amorph
