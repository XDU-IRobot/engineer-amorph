/**
 * @file ik_node.cpp
 * @brief ROS2 node: provides a draggable arrow (InteractiveMarker) in RViz
 *        representing the end-effector target pose. When dragged, computes
 *        inverse kinematics and publishes joint angles to /joint_states.
 *
 * Parameters (all optional):
 *   a2       (double, default: 0.4235)   Upper-arm length [m]
 *   a3       (double, default: 0.0)      Elbow x-offset [m]
 *   d3       (double, default: 0.0)      Shoulder z-offset [m]
 *   d4       (double, default: 0.587)    Forearm length [m]
 *   frame_id (string, default: "base_link_l")  RViz fixed frame
 */

#include <chrono>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "amorph_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

class IKNode : public rclcpp::Node
{
public:
  IKNode()
  : Node("ik_node"),
    marker_server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "ik_target", this->get_node_base_interface(),
      this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_topics_interface(), this->get_node_services_interface()))
  {
    // Declare kinematic parameters
    this->declare_parameter("a2", 0.4235);
    this->declare_parameter("a3", 0.0);
    this->declare_parameter("d3", 0.0);
    this->declare_parameter("d4", 0.587);
    this->declare_parameter("frame_id", "base_link_l");

    updateKinematics();

    // Publisher: joint states (IK result)
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Current joint state (for picking closest IK solution)
    current_joints_.fill(0.0);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IKNode::jointStateCallback, this, std::placeholders::_1));

    // Create the interactive marker (arrow at default pose)
    createInteractiveMarker();

    // Timer to keep publishing /joint_states at ~50 Hz
    timer_ = this->create_wall_timer(20ms, std::bind(&IKNode::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(),
      "IK node started. Drag the arrow in RViz to set target pose.");
  }

private:
  void updateKinematics()
  {
    double a2 = this->get_parameter("a2").as_double();
    double a3 = this->get_parameter("a3").as_double();
    double d3 = this->get_parameter("d3").as_double();
    double d4 = this->get_parameter("d4").as_double();
    kinematics_ = std::make_unique<amorph_kinematics::ArmKinematics>(a2, a3, d3, d4);

    RCLCPP_INFO(this->get_logger(),
      "Kinematics parameters: a2=%.4f a3=%.4f d3=%.4f d4=%.4f", a2, a3, d3, d4);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string & n = msg->name[i];
      if (n == "j1_l") { current_joints_[0] = msg->position[i]; }
      else if (n == "j2_l") { current_joints_[1] = msg->position[i]; }
      else if (n == "j3_l") { current_joints_[2] = msg->position[i]; }
      else if (n == "j4_l") { current_joints_[3] = msg->position[i]; }
      else if (n == "j5_l") { current_joints_[4] = msg->position[i]; }
      else if (n == "j6_l") { current_joints_[5] = msg->position[i]; }
    }
  }

  void createInteractiveMarker()
  {
    const std::string frame_id = this->get_parameter("frame_id").as_string();

    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = frame_id;
    im.name = "ik_target_arrow";
    im.description = "IK Target (drag axes to translate, rings to rotate)";
    im.scale = 0.15;

    // ---- Arrow visual (0.12 m length, with explicit start/end) ----
    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    // shaft diameter = 4 mm, head diameter = 8 mm
    arrow.scale.x = 0.004;
    arrow.scale.y = 0.008;
    arrow.scale.z = 0.0;
    arrow.color.r = 0.2f;
    arrow.color.g = 0.6f;
    arrow.color.b = 1.0f;
    arrow.color.a = 1.0f;
    geometry_msgs::msg::Point s, e;
    s.x = 0.0;  s.y = 0.0;  s.z = 0.0;
    e.x = 0.12; e.y = 0.0;  e.z = 0.0;
    arrow.points = {s, e};

    // The arrow is drawn along the marker's local +X.
    // Add a small sphere at the tip so it's clear which direction is forward.
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
    sphere.color.r = 0.2f;
    sphere.color.g = 0.6f;
    sphere.color.b = 1.0f;
    sphere.color.a = 1.0f;
    sphere.pose.position.x = 0.12;

    // ---- Controls ----
    // 1) 6-DOF control with just the visual (always visible)
    visualization_msgs::msg::InteractiveMarkerControl view_control;
    view_control.always_visible = true;
    view_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
    view_control.markers.push_back(arrow);
    view_control.markers.push_back(sphere);
    im.controls.push_back(view_control);

    // 2) Axis translation controls
    auto addMoveControl = [&](int axis, const std::string & name) {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = name;
      if (axis == 0) {
        ctrl.orientation.w = 1.0;
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      } else if (axis == 1) {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2));
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      } else {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI_2));
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      }
      im.controls.push_back(ctrl);
    };
    addMoveControl(0, "move_x");
    addMoveControl(1, "move_y");
    addMoveControl(2, "move_z");

    // 3) Axis rotation controls
    auto addRotateControl = [&](int axis, const std::string & name) {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = name;
      if (axis == 0) {
        ctrl.orientation.w = 1.0;
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      } else if (axis == 1) {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2));
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      } else {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI_2));
        ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      }
      im.controls.push_back(ctrl);
    };
    addRotateControl(0, "rotate_x");
    addRotateControl(1, "rotate_y");
    addRotateControl(2, "rotate_z");

    // Default pose
    im.pose.position.x = 0.35;
    im.pose.position.y = 0.0;
    im.pose.position.z = 0.4;
    im.pose.orientation.w = 1.0;

    marker_server_->insert(im);
    marker_server_->setCallback(
      im.name,
      std::bind(&IKNode::markerFeedback, this, std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);

    marker_server_->applyChanges();

    // Immediately compute IK for the default pose
    solveAndPublish(targetFromMarker(im.pose));
  }

  void markerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      return;
    }

    solveAndPublish(targetFromMarker(feedback->pose));

    // Update the marker pose (keep it where the user dragged it)
    marker_server_->setPose("ik_target_arrow", feedback->pose);
    marker_server_->applyChanges();
  }

  void poseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d & T) const
  {
    Eigen::Quaterniond q(pose.orientation.w,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z);
    T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;
  }

  /// Convert marker pose to IK target T0_6.
  /// The arrow visual is along +X, but the kinematics model's end-effector
  /// approach direction is +Z.  Compensate with R_y(-π/2).
  Eigen::Matrix4d targetFromMarker(const geometry_msgs::msg::Pose & marker_pose) const
  {
    Eigen::Matrix4d T;
    poseToMatrix(marker_pose, T);
    // R_y(+π/2): X→+Z, Y→Y, Z→-X
    T.block<3,3>(0,0) = T.block<3,3>(0,0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return T;
  }

  void solveAndPublish(const Eigen::Matrix4d & T_target)
  {
    auto solutions = kinematics_->inverseKinematics(T_target);
    solutions = amorph_kinematics::ArmKinematics::filterForwardElbowUp(solutions);

    if (solutions.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "IK: no solution for target pose");
      return;
    }

    // Pick solution closest to current joint state
    Eigen::Matrix<double, 6, 1> q_cur;
    q_cur << current_joints_[0], current_joints_[1], current_joints_[2],
             current_joints_[3], current_joints_[4], current_joints_[5];
    Eigen::Matrix<double, 6, 1> cur_theta =
      amorph_kinematics::ArmKinematics::urdfToModel(q_cur);

    int best_idx = 0;
    double best_dist = (solutions[0] - cur_theta).norm();
    for (size_t i = 1; i < solutions.size(); ++i) {
      double dist = (solutions[i] - cur_theta).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = static_cast<int>(i);
      }
    }

    ik_theta_ = solutions[best_idx];

    // Convert model theta back to URDF joint angles
    Eigen::Matrix<double, 6, 1> q_out =
      amorph_kinematics::ArmKinematics::modelToUrdf(ik_theta_);
    for (int j = 0; j < 6; ++j) { ik_urdf_joints_[j] = q_out(j); }

    ik_ready_ = true;
  }

  void publishJointStates()
  {
    if (!ik_ready_) return;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"j1_l", "j2_l", "j3_l", "j4_l", "j5_l", "j6_l"};
    msg.position = {ik_urdf_joints_[0], ik_urdf_joints_[1], ik_urdf_joints_[2],
                    ik_urdf_joints_[3], ik_urdf_joints_[4], ik_urdf_joints_[5]};
    joint_pub_->publish(msg);
  }

  // Kinematics
  std::unique_ptr<amorph_kinematics::ArmKinematics> kinematics_;

  // Interactive marker
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

  // Subscriptions / Publications
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::array<double, 6> current_joints_{};
  Eigen::Matrix<double, 6, 1> ik_theta_;
  std::array<double, 6> ik_urdf_joints_{};
  bool ik_ready_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
