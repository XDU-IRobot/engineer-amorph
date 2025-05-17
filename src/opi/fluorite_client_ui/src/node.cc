
#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rm2_referee_msgs/srv/tx.hpp>
#include <fluorite_msgs/msg/arm_state.hpp>

#include "ui.h"
#include "arm_renderer.hpp"
#include "ui_main.h"

extern "C" {
using MsgTxCallback = void (*)(const uint8_t *, int);
MsgTxCallback send_message_fn{nullptr};
}

MsgTxCallback StdFunctionToFnPtr(std::function<void(const uint8_t *, int)> fn) {
  static auto fn_static = std::move(fn);
  return [](const uint8_t *message, int length) { fn_static(message, length); };
}

class FluoriteClientUINode : public rclcpp::Node {
 public:
  explicit FluoriteClientUINode(
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : rclcpp::Node("fluorite_client_ui", options) {
    referee_tx_client_ = create_client<rm2_referee_msgs::srv::Tx>("/rm2_referee/tx");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(), [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
          static ArmRenderer renderer;
          auto lines = renderer.draw(msg->position[0], msg->position[1], msg->position[2]);
          {
            constexpr int kUIOffsetX = 0;
            constexpr int kUIOffsetY = 0;
            std::lock_guard lock(ui_data_mutex_);
            ui_main_static_yaw->start_x = lines[0].first.x + kUIOffsetX;
            ui_main_static_yaw->start_y = lines[0].first.y + kUIOffsetY;
            ui_main_static_yaw->end_x = lines[0].second.x + kUIOffsetX;
            ui_main_static_yaw->end_y = lines[0].second.y + kUIOffsetY;
            ui_main_dynamic_pitch1->start_x = lines[0].first.x + kUIOffsetX;
            ui_main_dynamic_pitch1->start_y = lines[0].first.y + kUIOffsetY;
            ui_main_dynamic_pitch1->end_x = lines[0].second.x + kUIOffsetX;
            ui_main_dynamic_pitch1->end_y = lines[0].second.y + kUIOffsetY;
            ui_main_dynamic_pitch2->start_x = lines[1].first.x + kUIOffsetX;
            ui_main_dynamic_pitch2->start_y = lines[1].first.y + kUIOffsetY;
            ui_main_dynamic_pitch2->end_x = lines[1].second.x + kUIOffsetX;
            ui_main_dynamic_pitch2->end_y = lines[1].second.y + kUIOffsetY;
          }
        });
    arm_state_sub_ = create_subscription<fluorite_msgs::msg::ArmState>(
        "/fluorite/blackbox/arm_state", rclcpp::SystemDefaultsQoS(),
        [&](const fluorite_msgs::msg::ArmState::SharedPtr msg) {
          std::lock_guard lock(ui_data_mutex_);
          ui_main_dynamic_arm_pump->color = msg->arm_pump ? 2 : 7;
          ui_main_dynamic_left_storage->color = msg->left_storage ? 2 : 7;
          ui_main_dynamic_right_storage->color = msg->right_storage ? 2 : 7;

          switch (msg->fsm_state) {
            case fluorite_msgs::msg::ArmState::FSM_STATE_IDLE: {
              std::strcpy(ui_main_dynamic_arm_status->string, "");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_EXEC: {
              std::strcpy(ui_main_dynamic_arm_status->string, "EXECUTING");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_MANUAL: {
              std::strcpy(ui_main_dynamic_arm_status->string, "MANUAL");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_NOFORCE: {
              std::strcpy(ui_main_dynamic_arm_status->string, "NOFORCE");
              break;
            }
          }
        });
    tx_timer_ = create_wall_timer(std::chrono::milliseconds(50), [&] {
      std::lock_guard lock(ui_data_mutex_);
      ui_update_main();
    });
    send_message_fn = StdFunctionToFnPtr([&](const uint8_t *message, int length) {
      auto request = std::make_shared<rm2_referee_msgs::srv::Tx::Request>();
      request->data.resize(length);
      memcpy(request->data.data(), message, length);
      if (!referee_tx_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "Service not available after waiting");
        return;
      }
      auto result = referee_tx_client_->async_send_request(request);
      result.wait();  // wait for request being processed
    });
  }

 private:
  rclcpp::Client<rm2_referee_msgs::srv::Tx>::SharedPtr referee_tx_client_;
  rm2_referee_msgs::srv::Tx::Request referee_tx_request_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<fluorite_msgs::msg::ArmState>::SharedPtr arm_state_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_{};

  std::mutex ui_data_mutex_{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteClientUINode>());
  rclcpp::shutdown();
  return 0;
}