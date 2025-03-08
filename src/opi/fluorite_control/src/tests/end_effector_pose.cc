
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>

rclcpp::Node::SharedPtr mgi_node;
rclcpp::Executor::SharedPtr mgi_node_executor;
std::thread mgi_node_thread;

void mgi() {
  auto node = rclcpp::Node::make_shared("get_end_effector_pose");
  mgi_node = std::make_shared<rclcpp::Node>("move_group_interface_node");
  mgi_node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  mgi_node_executor->add_node(mgi_node);

  moveit::planning_interface::MoveGroupInterface move_group(mgi_node, "end");
  // move_group.startStateMonitor();

  mgi_node_thread = std::thread([&] { mgi_node_executor->spin(); });

  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

    RCLCPP_INFO(node->get_logger(),
                "End effector pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
  }
}

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
void tf() {
  auto node = rclcpp::Node::make_shared("get_end_effector_pose");
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  while (rclcpp::ok()) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer->lookupTransform("end_yaw2_link", "base_yaw_link", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
      continue;
    }

    RCLCPP_INFO(node->get_logger(),
                "End effector pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
                transform.transform.rotation.w);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  tf();
  rclcpp::shutdown();
  return 0;
}