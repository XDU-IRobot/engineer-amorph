/**
 * @file test_fk_ik_node.cpp
 * @brief Test node: combines IK + FK in a single node.
 *
 * Provides an interactive marker (IK target, blue arrow). On each drag:
 *  1. Solve IK → publish /joint_states
 *  2. Read back joints, run FK → publish FK arrow (green) to /fk_verify_arrow
 *
 * If both arrows coincide, IK ↔ FK consistency is verified.
 *
 * Parameters:
 *   a2, a3, d3, d4, frame_id — same as ik_node / fk_node
 */

#include <chrono>
#include <string>
#include <vector>
#include <array>
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

class TestFKIKNode : public rclcpp::Node
{
public:
  TestFKIKNode()
  : Node("test_fk_ik_node"),
    marker_server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "ik_test_target", this->get_node_base_interface(),
      this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_topics_interface(), this->get_node_services_interface()))
  {
    this->declare_parameter("a2", 0.479754);
    this->declare_parameter("a3", 0.0);
    this->declare_parameter("d3", 0.0);
    this->declare_parameter("d4", 0.45);
    this->declare_parameter("frame_id", "base_link_l");

    updateKinematics();

    // Publisher: joint states (IK result)
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Publisher: FK verification arrow (green)
    fk_arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/fk_verify_arrow", 10);

    // Subscriber: read back joint states for FK
    current_joints_.fill(0.0);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&TestFKIKNode::jointStateCallback, this, std::placeholders::_1));

    createInteractiveMarker();

    // Timer: run FK on current joints and publish arrow
    timer_ = this->create_wall_timer(20ms,
      std::bind(&TestFKIKNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "Test FK-IK node started. "
      "Blue arrow = IK target, Green arrow = FK result for current joints.");
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
      "Kinematics: a2=%.4f a3=%.4f d3=%.4f d4=%.4f", a2, a3, d3, d4);
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
    im.name = "ik_test_target_arrow";
    im.description = "IK Target (blue)";
    im.scale = 0.15;

    // Arrow visual
    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
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

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
    sphere.color.r = 0.2f;
    sphere.color.g = 0.6f;
    sphere.color.b = 1.0f;
    sphere.color.a = 1.0f;
    sphere.pose.position.x = 0.12;

    visualization_msgs::msg::InteractiveMarkerControl view_ctrl;
    view_ctrl.always_visible = true;
    view_ctrl.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
    view_ctrl.markers.push_back(arrow);
    view_ctrl.markers.push_back(sphere);
    im.controls.push_back(view_ctrl);

    // Axis controls
    auto addControl = [&](int axis, const std::string & name, int mode) {
      visualization_msgs::msg::InteractiveMarkerControl ctrl;
      ctrl.name = name;
      if (axis == 0) {
        ctrl.orientation.w = 1.0;
      } else if (axis == 1) {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2));
      } else {
        ctrl.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI_2));
      }
      ctrl.interaction_mode = mode;
      im.controls.push_back(ctrl);
    };
    addControl(0, "move_x",  visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
    addControl(1, "move_y",  visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
    addControl(2, "move_z",  visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
    addControl(0, "rot_x",   visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    addControl(1, "rot_y",   visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    addControl(2, "rot_z",   visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);

    im.pose.position.x = 0.35;
    im.pose.position.y = 0.0;
    im.pose.position.z = 0.4;
    im.pose.orientation.w = 1.0;

    marker_server_->insert(im);
    marker_server_->setCallback(
      im.name,
      std::bind(&TestFKIKNode::markerFeedback, this, std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
    marker_server_->applyChanges();

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
    marker_server_->setPose("ik_test_target_arrow", feedback->pose);
    marker_server_->applyChanges();
  }

  // ---- Coordinate utilities ----

  void poseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d & T) const
  {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;
  }

  Eigen::Matrix4d targetFromMarker(const geometry_msgs::msg::Pose & marker_pose) const
  {
    Eigen::Matrix4d T;
    poseToMatrix(marker_pose, T);
    T.block<3,3>(0,0) = T.block<3,3>(0,0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return T;
  }

  // ---- Inverse Kinematics ----

  void solveAndPublish(const Eigen::Matrix4d & T_target)
  {
    // SolveIk 内部已完成: 去重 → elbow-up 筛选 → URDF 转换
    auto solutions = kinematics_->SolveIk(T_target);
    if (solutions.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "IK: no solution after filtering");
      return;
    }

    // solutions 已为 URDF 空间，直接与当前 URDF 关节角比较
    Eigen::Matrix<double, 6, 1> q_cur;
    q_cur << current_joints_[0], current_joints_[1], current_joints_[2],
             current_joints_[3], current_joints_[4], current_joints_[5];

    int best = 0;
    double best_d = (solutions[0] - q_cur).norm();
    for (size_t i = 1; i < solutions.size(); ++i) {
      double d = (solutions[i] - q_cur).norm();
      if (d < best_d) { best_d = d; best = static_cast<int>(i); }
    }

    // 直接发布 URDF 关节角
    for (int j = 0; j < 6; ++j) { ik_urdf_[j] = solutions[best](j); }
    ik_ready_ = true;

    // Print all IK solutions (URDF angles in degrees)
    RCLCPP_INFO(this->get_logger(), "=== IK solved, %zu solution(s): ===", solutions.size());
    for (size_t i = 0; i < solutions.size(); ++i) {
      const auto deg = (solutions[i] * (180.0 / M_PI)).eval();
      const char * marker = (static_cast<int>(i) == best) ? " <-- selected" : "";
      RCLCPP_INFO(this->get_logger(),
        "  [%zu]  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f deg%s",
        i, deg(0), deg(1), deg(2), deg(3), deg(4), deg(5), marker);
    }
  }

  // ---- Timer callback: FK → green arrow ----

  void timerCallback()
  {
    // 1) Publish joint states
    if (ik_ready_) {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = this->now();
      msg.name = {"j1_l", "j2_l", "j3_l", "j4_l", "j5_l", "j6_l"};
      msg.position = {ik_urdf_[0], ik_urdf_[1], ik_urdf_[2],
                      ik_urdf_[3], ik_urdf_[4], ik_urdf_[5]};
      joint_pub_->publish(msg);
    }

    // 2) Run FK on current joints and publish green arrow
    Eigen::Matrix<double, 6, 1> q_cur2;
    q_cur2 << current_joints_[0], current_joints_[1], current_joints_[2],
              current_joints_[3], current_joints_[4], current_joints_[5];
    // SolveFk 内部已做 urdfToModel 转换
    Eigen::Matrix4d T = kinematics_->SolveFk(q_cur2);
    Eigen::Vector3d p = T.block<3,1>(0,3);
    Eigen::Vector3d z = T.block<3,1>(0,2);

    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = this->now();
    arrow.header.frame_id = this->get_parameter("frame_id").as_string();
    arrow.ns = "fk_verify";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.lifetime = rclcpp::Duration(0, 0);

    arrow.scale.x = 0.004;   // shaft
    arrow.scale.y = 0.008;   // head

    geometry_msgs::msg::Point start, end;
    double len = 0.12;
    start.x = p(0) - z(0) * len * 0.8;
    start.y = p(1) - z(1) * len * 0.8;
    start.z = p(2) - z(2) * len * 0.8;
    end.x   = p(0) + z(0) * len * 0.2;
    end.y   = p(1) + z(1) * len * 0.2;
    end.z   = p(2) + z(2) * len * 0.2;
    arrow.points = {start, end};

    arrow.color.r = 0.2f;
    arrow.color.g = 0.8f;
    arrow.color.b = 0.2f;
    arrow.color.a = 1.0f;

    fk_arrow_pub_->publish(arrow);
  }

  // Members
  std::unique_ptr<amorph_kinematics::ArmKinematics> kinematics_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fk_arrow_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> current_joints_{};
  std::array<double, 6> ik_urdf_{};
  bool ik_ready_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestFKIKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
