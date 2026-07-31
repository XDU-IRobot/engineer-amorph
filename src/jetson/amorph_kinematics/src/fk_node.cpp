/**
 * @file fk_node.cpp
 * @brief ROS2 node: reads joint_states, computes forward kinematics,
 *        publishes an arrow marker to RViz.
 *
 * Parameters (all optional, with defaults approximating the amorph arm):
 *   a2       (double, default: 0.42352)  Upper-arm length [m]
 *   a3       (double, default: 0.147)    Elbow x-offset [m]
 *   d3       (double, default: 0.0193)   Shoulder z-offset [m]
 *   d4       (double, default: 0.44)     Forearm length [m]
 *   frame_id (string, default: "base_link_l")  RViz fixed frame
 */

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "amorph_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

class FKNode : public rclcpp::Node
{
public:
  FKNode()
  : Node("fk_node")
  {
    // Declare kinematic parameters
    this->declare_parameter("a2", 0.4235);
    this->declare_parameter("a3", 0.0);
    this->declare_parameter("d3", 0.0);
    this->declare_parameter("d4", 0.587);
    this->declare_parameter("frame_id", "base_link_l");

    // Create kinematics solver
    updateKinematics();

    // Publisher: arrow marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "fk_arrow", 10);

    // Subscriber: joint states
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FKNode::jointStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "FK node started. Publishing arrow to '%s' in frame '%s'",
      marker_pub_->get_topic_name(), this->get_parameter("frame_id").as_string().c_str());
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
    // Look up j1_l .. j6_l
    double jv[6];
    bool found[6] = {false};

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string & n = msg->name[i];
      if (n == "j1_l") { jv[0] = msg->position[i]; found[0] = true; }
      else if (n == "j2_l") { jv[1] = msg->position[i]; found[1] = true; }
      else if (n == "j3_l") { jv[2] = msg->position[i]; found[2] = true; }
      else if (n == "j4_l") { jv[3] = msg->position[i]; found[3] = true; }
      else if (n == "j5_l") { jv[4] = msg->position[i]; found[4] = true; }
      else if (n == "j6_l") { jv[5] = msg->position[i]; found[5] = true; }
    }

    // Skip if not all 6 joints are present
    for (int i = 0; i < 6; ++i) {
      if (!found[i]) return;
    }

    // Compute forward kinematics (convert URDF joints → model theta)
    Eigen::Matrix<double, 6, 1> q;
    q << jv[0], jv[1], jv[2], jv[3], jv[4], jv[5];
    Eigen::Matrix4d T = kinematics_->forwardKinematics(
      amorph_kinematics::ArmKinematics::urdfToModel(q));

    // Publish arrow marker
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = this->get_parameter("frame_id").as_string();
    marker.ns = "fk_end_effector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(0, 0);  // persistent

    // Start point (wrist position, slightly back along approach)
    Eigen::Vector3d p = T.block<3,1>(0,3);
    Eigen::Vector3d z_axis = T.block<3,1>(0,2);  // approach direction (z6)

    double arrow_len = 0.1;  // 10 cm arrow

    geometry_msgs::msg::Point start, end;
    start.x = p(0) - z_axis(0) * arrow_len * 0.8;
    start.y = p(1) - z_axis(1) * arrow_len * 0.8;
    start.z = p(2) - z_axis(2) * arrow_len * 0.8;

    end.x = p(0) + z_axis(0) * arrow_len * 0.2;
    end.y = p(1) + z_axis(1) * arrow_len * 0.2;
    end.z = p(2) + z_axis(2) * arrow_len * 0.2;

    marker.points = {start, end};

    marker.scale.x = 0.005;  // shaft diameter
    marker.scale.y = 0.01;   // head diameter
    marker.scale.z = 0.0;

    marker.color.r = 1.0f;
    marker.color.g = 0.3f;
    marker.color.b = 0.3f;
    marker.color.a = 1.0f;

    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<amorph_kinematics::ArmKinematics> kinematics_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
