#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <librm.hpp>

#include "amorph_transport/schema.hpp"
#include "amorph_transport/stm32.hpp"

using namespace std::chrono_literals;

namespace amorph {

class TransportNode : public rclcpp::Node {
 public:
  explicit TransportNode(const rclcpp::NodeOptions &options)
      : Node("amorph_transport_node", options) {
    // ---- 参数 ----
    this->declare_parameter("can_dev", "can0");
    this->declare_parameter("publish_rate", 45.0);  // Hz

    const auto can_dev = this->get_parameter("can_dev").as_string();
    const auto rate = this->get_parameter("publish_rate").as_double();

    // ---- joint 名称 (与 URDF 中一致) ----
    joint_names_ = {
        "j1_l", "j2_l", "j3_l", "j4_l", "j5_l", "j6_l",
        "j1_r", "j2_r", "j3_r", "j4_r", "j5_r", "j6_r",
        "gripper_l_l", "gripper_r_l",
        "gripper_l_r", "gripper_r_r",
    };

    // ---- CAN 初始化 ----
    try {
      can_ = std::make_unique<rm::hal::Can>(can_dev.c_str());
      RCLCPP_INFO(this->get_logger(), "CAN device '%s' created", can_dev.c_str());
    } catch (const std::exception &e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open CAN device '%s': %s",
                   can_dev.c_str(), e.what());
      throw;
    }

    stm32_ = std::make_unique<STM32>(*can_);
    can_->Begin();

    // ---- publisher ----
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // ---- 定时发布 ----
    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
        period, std::bind(&TransportNode::PublishCallback, this));

    RCLCPP_INFO(this->get_logger(), "TransportNode ready, CAN=%s, rate=%.1fHz",
                can_dev.c_str(), rate);
  }

  ~TransportNode() override {
    if (can_) {
      can_->Stop();
    }
  }

 private:
  void PublishCallback() {
    auto maybe_state = stm32_->GetLatest();
    if (!maybe_state.has_value()) {
      return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();

    // 左臂 6 轴 + 右臂 6 轴 + 4 夹爪
    msg.name.resize(16);
    msg.position.resize(16);

    msg.name = joint_names_;

    const auto &s = maybe_state.value();
    for (int i = 0; i < 6; i++) {
      msg.position[i]     = s.left[i];
      msg.position[6 + i] = s.right[i];
    }
    // 夹爪: bool → 0.0 / 0.04 (米)
    msg.position[12] = s.left_gripper_open  ? 0.04f : 0.0f;
    msg.position[13] = s.left_gripper_open  ? 0.04f : 0.0f;
    msg.position[14] = s.right_gripper_open ? 0.04f : 0.0f;
    msg.position[15] = s.right_gripper_open ? 0.04f : 0.0f;

    joint_pub_->publish(msg);
  }

  // ---- 成员 ----
  std::unique_ptr<rm::hal::Can> can_;
  std::unique_ptr<STM32> stm32_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
};

}  // namespace amorph

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amorph::TransportNode)
