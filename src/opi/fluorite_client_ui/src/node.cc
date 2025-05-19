
#include <chrono>
#include <cstring>
#include <deque>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rm2_referee_msgs/msg/robot_status.hpp>
#include <rm2_referee_msgs/msg/remote_control.hpp>
#include <rm2_referee_msgs/srv/tx.hpp>
#include <fluorite_msgs/msg/arm_state.hpp>

#include "ui.h"
#include "arm_renderer.hpp"

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
  std::pair<Point, Point> line1_coord_, line2_coord_, line3_coord_;
  explicit FluoriteClientUINode(
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : rclcpp::Node("fluorite_client_ui", options) {
    referee_tx_client_ = create_client<rm2_referee_msgs::srv::Tx>("/rm2_referee/tx");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
          static ArmRenderer renderer;
          // RCLCPP_INFO(get_logger(), "joint positions: %f %f %f", msg->position[0], msg->position[1],
          // msg->position[2]);
          const auto lines = renderer.Draw(msg->position[0], msg->position[1], msg->position[2]);
          line1_coord_ = lines[0];
          line2_coord_ = lines[1];
          line3_coord_ = lines[2];

          std::ostringstream oss;

          constexpr int kUIOffsetX = 580;
          constexpr int kUIOffsetY = 240;
          // std::lock_guard lock(ui_data_mutex_);
          ui_main_static_yaw->start_x = line1_coord_.first.x + kUIOffsetX;
          ui_main_static_yaw->start_y = line1_coord_.first.y + kUIOffsetY;
          ui_main_static_yaw->end_x = line1_coord_.second.x + kUIOffsetX;
          ui_main_static_yaw->end_y = line1_coord_.second.y + kUIOffsetY;
          ui_main_dynamic_pitch1->start_x = line2_coord_.first.x + kUIOffsetX;
          ui_main_dynamic_pitch1->start_y = line2_coord_.first.y + kUIOffsetY;
          ui_main_dynamic_pitch1->end_x = line2_coord_.second.x + kUIOffsetX;
          ui_main_dynamic_pitch1->end_y = line2_coord_.second.y + kUIOffsetY;
          ui_main_dynamic_pitch2->start_x = line3_coord_.first.x + kUIOffsetX;
          ui_main_dynamic_pitch2->start_y = line3_coord_.first.y + kUIOffsetY;
          ui_main_dynamic_pitch2->end_x = line3_coord_.second.x + kUIOffsetX;
          ui_main_dynamic_pitch2->end_y = line3_coord_.second.y + kUIOffsetY;
          // oss << "rendered lines:\n"
          //     << "  0: (" << ui_main_static_yaw->start_x << ", " << ui_main_static_yaw->start_y << ") -> ("
          //     << ui_main_static_yaw->end_x << ", " << ui_main_static_yaw->end_y << ")\n"
          //     << "  1: (" << ui_main_dynamic_pitch1->start_x << ", " << ui_main_dynamic_pitch1->start_y << ") -> ("
          //     << ui_main_dynamic_pitch1->end_x << ", " << ui_main_dynamic_pitch1->end_y << ")\n"
          //     << "  2: (" << ui_main_dynamic_pitch2->start_x << ", " << ui_main_dynamic_pitch2->start_y << ") -> ("
          //     << ui_main_dynamic_pitch2->end_x << ", " << ui_main_dynamic_pitch2->end_y << ")\n";
          // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        });
    arm_state_sub_ = create_subscription<fluorite_msgs::msg::ArmState>(
        "/fluorite/blackbox/arm_state", rclcpp::SystemDefaultsQoS(),
        [&](const fluorite_msgs::msg::ArmState::SharedPtr msg) {
          // std::lock_guard lock(ui_data_mutex_);
          ui_main_dynamic_arm_pump->color = [&] {
            switch (msg->arm_pump) {
              case fluorite_msgs::msg::ArmState::ARM_PUMP_STATE_OFF:
                return 7;
              case fluorite_msgs::msg::ArmState::ARM_PUMP_STATE_SUCK:
                return 2;
              case fluorite_msgs::msg::ArmState::ARM_PUMP_STATE_BLOW:
                return 3;
              default:
                return 7;
            }
          }();
          ui_main_dynamic_left_storage->color = msg->left_storage ? 2 : 7;
          ui_main_dynamic_right_storage->color = msg->right_storage ? 2 : 7;

          switch (msg->fsm_state) {
            case fluorite_msgs::msg::ArmState::FSM_STATE_IDLE: {
              std::strcpy(ui_main_dynamic_arm_status->string, "         ");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_EXEC: {
              std::strcpy(ui_main_dynamic_arm_status->string, "EXECUTING");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_MANUAL: {
              std::strcpy(ui_main_dynamic_arm_status->string, "MANUAL   ");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_NOFORCE: {
              std::strcpy(ui_main_dynamic_arm_status->string, "NOFORCE  ");
              break;
            }
            case fluorite_msgs::msg::ArmState::FSM_STATE_RETRACTED: {
              std::strcpy(ui_main_dynamic_arm_status->string, "RETRACTED");
              break;
            }
          }
        });
    robot_status_sub_ = create_subscription<rm2_referee_msgs::msg::RobotStatus>(
        "/rm2_referee/robot_status", rclcpp::SensorDataQoS(),
        [&](const rm2_referee_msgs::msg::RobotStatus::SharedPtr msg) { ui_self_id = msg->robot_id; });
    static bool last_fuck = false;
    keyboard_sub_ = create_subscription<rm2_referee_msgs::msg::RemoteControl>(
        "/rm2_referee/remote_control", rclcpp::SensorDataQoS(),
        [&](const rm2_referee_msgs::msg::RemoteControl::SharedPtr msg) {
          if ((msg->keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL) && !last_fuck) {
            ui_init_main();
            RCLCPP_INFO(get_logger(), "UI reinitialized!");
          }
          last_fuck = static_cast<bool>(msg->keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL);
        });
    send_message_fn = StdFunctionToFnPtr([&](const uint8_t *message, int length) {
      std::vector<uint8_t> data(length);
      std::copy(message, message + length, data.begin());
      tx_queue_.push_back(data);

      constexpr size_t kMaxTxQueueSize = 10;
      if (tx_queue_.size() > kMaxTxQueueSize) {
        RCLCPP_WARN(get_logger(), "Tx queue size exceeded, dropping oldest message");
        tx_queue_.pop_front();
      }
    });
    ui_update_timer_ = create_wall_timer(std::chrono::milliseconds(200), [&] { ui_update_main(); });
    tx_timer_ = create_wall_timer(std::chrono::milliseconds(45), [&] {
      // take a message from queue
      if (!tx_queue_.empty()) {
        if (!referee_tx_client_->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_ERROR(get_logger(), "Service not available after waiting");
          return;
        }
        auto request = std::make_shared<rm2_referee_msgs::srv::Tx::Request>();
        request->data = tx_queue_.front();
        tx_queue_.pop_front();
        auto result = referee_tx_client_->async_send_request(request);
      } else {
      }
    });
    ui_self_id = 102;
    ui_init_main();
  }

 private:
  bool ui_initialized_{false};
  rclcpp::Client<rm2_referee_msgs::srv::Tx>::SharedPtr referee_tx_client_;
  rm2_referee_msgs::srv::Tx::Request referee_tx_request_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<rm2_referee_msgs::msg::RemoteControl>::SharedPtr keyboard_sub_;
  rclcpp::Subscription<rm2_referee_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<fluorite_msgs::msg::ArmState>::SharedPtr arm_state_sub_;
  rclcpp::TimerBase::SharedPtr ui_update_timer_{};
  rclcpp::TimerBase::SharedPtr tx_timer_{};
  std::deque<std::vector<uint8_t>> tx_queue_{};

  // std::mutex ui_data_mutex_{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteClientUINode>());
  rclcpp::shutdown();
  return 0;
}