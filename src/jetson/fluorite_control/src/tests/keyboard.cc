
#include <rclcpp/rclcpp.hpp>

#include "keyboard_msgs/msg/key.hpp"

class KeyboardSubNode : public rclcpp::Node {
 public:
  KeyboardSubNode() : Node("keyboard_sub_node") {
    keyboard_sub_ = create_subscription<keyboard_msgs::msg::Key>(
        "/keydown", rclcpp::SensorDataQoS(), [&](const keyboard_msgs::msg::Key::SharedPtr msg) {
          RCLCPP_INFO(get_logger(), "Key code: %d, modifier: %d", msg->code, msg->modifiers);
        });
  }

 private:
  rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyboard_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardSubNode>());
  rclcpp::shutdown();
  return 0;
}