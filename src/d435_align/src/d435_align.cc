/**
 * @file   d435_align.cc
 * @brief  发布静态tf，把d435的坐标系与moveit里机械臂的坐标系对齐
 */

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

using namespace std::chrono_literals;

class D435AlignNode : public rclcpp::Node {
 public:
  D435AlignNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                                  .allow_undeclared_parameters(true)
                                                  .automatically_declare_parameters_from_overrides(true))
      : Node("d435_align", options) {
    get_parameter("world_frame", frame_id_.world_frame);
    get_parameter("d435_frame", frame_id_.d435_frame);
    get_parameter("arm_frame", frame_id_.arm_frame);
    RCLCPP_INFO(get_logger(), "world_frame: %s, d435_frame: %s, arm_frame: %s", frame_id_.world_frame.c_str(),
                frame_id_.d435_frame.c_str(), frame_id_.arm_frame.c_str());

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // world to d435
    static_tf_world_to_d435.header.frame_id = frame_id_.world_frame;
    static_tf_world_to_d435.child_frame_id = frame_id_.d435_frame;
    static_tf_world_to_d435.transform.translation.x = 0.0;
    static_tf_world_to_d435.transform.translation.y = 0.0;
    static_tf_world_to_d435.transform.translation.z = 0.0;
    static_tf_world_to_d435.transform.rotation.x = 0.0;
    static_tf_world_to_d435.transform.rotation.y = 0.0;
    static_tf_world_to_d435.transform.rotation.z = 0.0;
    static_tf_world_to_d435.transform.rotation.w = 1.0;

    // world to arm
    static_tf_world_to_arm.header.frame_id = frame_id_.world_frame;
    static_tf_world_to_arm.child_frame_id = frame_id_.arm_frame;
    static_tf_world_to_arm.transform.translation.x = 0.0;
    static_tf_world_to_arm.transform.translation.y = 0.0;
    static_tf_world_to_arm.transform.translation.z = 0.0;
    static_tf_world_to_arm.transform.rotation.x = 0.0;
    static_tf_world_to_arm.transform.rotation.y = 0.0;
    static_tf_world_to_arm.transform.rotation.z = 0.0;
    static_tf_world_to_arm.transform.rotation.w = 1.0;

    timer_ = create_wall_timer(1s, [this]() {
      static_tf_world_to_d435.header.stamp = now();
      static_tf_world_to_arm.header.stamp = now();
      static_tf_broadcaster_->sendTransform(static_tf_world_to_d435);
      static_tf_broadcaster_->sendTransform(static_tf_world_to_arm);
    });
  }

 private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped static_tf_world_to_d435{};
  geometry_msgs::msg::TransformStamped static_tf_world_to_arm{};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  struct {
    std::string world_frame{};
    std::string d435_frame{};
    std::string arm_frame{};
  } frame_id_{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D435AlignNode>());
  rclcpp::shutdown();
  return 0;
}