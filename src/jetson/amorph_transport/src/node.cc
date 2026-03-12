
#include "node.hpp"

#include "host_comm.hpp"
#include "logger.hpp"

namespace amorph {

using namespace text::literals;

TransportNode::TransportNode(const rclcpp::NodeOptions &options)
    : Node("arm_controller", rclcpp::NodeOptions()                   //
                                 .allow_undeclared_parameters(true)  //
                                 .automatically_declare_parameters_from_overrides(true)) {
  params_manager_.Init();
  InitComm();
  InitActionServers();
  L::I("node initialized."_green);
}

void TransportNode::InitComm() {
  const std::string can_port = params_manager_.data().rpc.can_port;
  const int mosi_frame_id = params_manager_.data().rpc.mosi_frame_id;
  const int miso_frame_id = params_manager_.data().rpc.miso_frame_id;

  comm_can_ = std::make_unique<rm::hal::Can>(can_port.c_str());
  stm32_ = std::make_unique<Stm32>(*comm_can_, miso_frame_id);

  static rm::device::DeviceManager<1> stm32_connection_watcher;
  stm32_connection_watcher << stm32_.get();
  stm32_connection_watcher.OnDeviceStatusChange([](rm::device::Device *device) {
    if (device->online_status() == rm::device::Device::kOk) {
      L::I("STM32 connection established."_green);
    } else {
      L::W("STM32 connection lost."_yellow);
    }
  });

  // 订阅关节位置，发布JointState
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(  //
      params_manager_.data().joint_state_topic,                       //
      10                                                              //
  );
  static sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.name = params_manager_.data().joint_names;
  stm32_->OnRx([&](const Stm32Feedback &data) {
    joint_state_msg.position.clear();
    joint_state_msg.position.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_state_msg.position.push_back(data.joint_positions[i]);
    }
    joint_state_msg.position.push_back(data.gripper_open ? 0.0 : -0.02);  // 夹爪开=0，夹爪闭=-0.02
    joint_state_msg.velocity.clear();
    joint_state_msg.velocity.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_state_msg.velocity.push_back(data.joint_velocities[i]);
    }
    joint_state_msg.velocity.push_back(0.f);
    joint_state_msg.effort.clear();
    joint_state_msg.effort.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_state_msg.effort.push_back(data.joint_efforts[i]);
    }
    joint_state_msg.effort.push_back(0.f);
    joint_state_msg.header.stamp = rclcpp::Clock().now();
    joint_state_pub_->publish(joint_state_msg);
  });

  comm_can_->Begin();

  joint_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/transport/joint_command", 10);

  stm32_tx_timer_ = create_wall_timer(std::chrono::milliseconds(5), [&]() {
    stm32_connection_watcher.Update();

    // 更新轨迹跟踪器，将插值结果写入 STM32 发送缓冲区
    trajectory_tracker_.Update();
    if (trajectory_tracker_.is_initialized()) {
      const auto pt = trajectory_tracker_.current_point();
      std_msgs::msg::Float64MultiArray cmd_msg;
      cmd_msg.data.resize(6);
      for (size_t i = 0; i < 6; ++i) {
        stm32_->tx_data_.target_joint_positions[i] = static_cast<float>(pt.positions[i]);
        cmd_msg.data[i] = pt.positions[i];
      }
      joint_command_pub_->publish(cmd_msg);
    }

    stm32_->Write();
  });

  L::I("Comm with stm32 ok."_green);
}

void TransportNode::InitActionServers() {
  arm_action_server_ =
      std::make_unique<ArmActionServer>(this, params_manager_.data().arm_action_server_name,
                                        params_manager_.data().joint_names, *stm32_, trajectory_tracker_);
  gripper_action_server_ =
      std::make_unique<GripperActionServer>(this, params_manager_.data().gripper_action_server_name);
  execute_task_solution_server_ = std::make_unique<ExecuteTaskSolutionServer>(this, *stm32_, trajectory_tracker_);
  L::I("Action servers ok."_green);
}

}  // namespace amorph

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amorph::TransportNode)
