
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <sbus_interface/msg/sbus.hpp>
#include <rm2_referee_msgs/msg/remote_control.hpp>

#include "bt_node_register.hpp"
#include "keypress_handler.hpp"
#include "tree_loader.hpp"
#include "moveit.hpp"

class AsyncTreeRunner {
 public:
  ~AsyncTreeRunner() {
    if (tick_worker_thread_.joinable()) {
      stop_flag_ = true;
      tick_worker_thread_.join();
    }
  }

  void Run(BT::Tree tree) {
    if (!stop_flag_) {
      RCLCPP_WARN(rclcpp::get_logger("bt_executor"), "Tree is already running");
      return;
    }
    if (tick_worker_thread_.joinable()) {
      tick_worker_thread_.join();
    }
    stop_flag_ = false;
    tree_ = std::move(tree);
    cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    zmq_logger_ = std::make_unique<BT::PublisherZMQ>(tree_);
    tick_worker_thread_ = std::thread([this]() {
      try {
        while (!stop_flag_) {
          const auto tree_status = tree_.tickRoot();
          if (tree_status == BT::NodeStatus::RUNNING) {
            // do nothing
          } else if (tree_status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("bt_executor"), "Tree finished successfully");
            Stop(false);
          } else if (tree_status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(rclcpp::get_logger("bt_executor"), "Tree failed");
            Stop(false);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_executor"), "Exception in tick worker: %s", e.what());
        Stop(false);
      }
    });
  }

  void Stop(bool join_worker = true) {
    stop_flag_ = true;
    cout_logger_.reset();
    zmq_logger_.reset();
    tree_.haltTree();
    if (tick_worker_thread_.joinable() && join_worker) {
      tick_worker_thread_.join();
    }
  }

  bool have_ongoing_tree() { return !stop_flag_; }

 private:
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_{nullptr};
  std::unique_ptr<BT::PublisherZMQ> zmq_logger_{nullptr};

  std::thread tick_worker_thread_;
  std::atomic<bool> stop_flag_{true};
};

class BTExecutorNode : public rclcpp::Node {
 public:
  explicit BTExecutorNode(
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : Node("bt_executor_node", options) {
    bt_nodes::BtNodeRegisterArray::Register(bt_factory_);

    // spawn user input subscribers
    sbus_sub_ = create_subscription<sbus_interface::msg::Sbus>(
        "/sbus", rclcpp::SensorDataQoS(), [&](const sbus_interface::msg::Sbus::SharedPtr msg) {
          constexpr int kRcSwitchUp = 1;
          constexpr int kRcSwitchMiddle = 0;
          constexpr int kRcSwitchDown = -1;
          if (!msg->failsafe && -msg->mapped_channels[10] / 1000 == kRcSwitchUp) {
            ready_to_command_ = true;
          } else {
            ready_to_command_ = false;
          }
        });
    fpv_link_input_sub_ = create_subscription<rm2_referee_msgs::msg::RemoteControl>(
        "/rm2_referee/remote_control", rclcpp::SensorDataQoS(),
        [&](const rm2_referee_msgs::msg::RemoteControl::SharedPtr msg) {
          hotkeys_.rise_eff.Poll(msg->keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_R);
          hotkeys_.go_to_left_storage.Poll(msg->keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_Z);
          hotkeys_.go_to_right_storage.Poll(msg->keyboard_value & rm2_referee_msgs::msg::RemoteControl::KEY_X);
        });

    // define keymap
    hotkeys_.rise_eff.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Rise effect key pressed");
      if (ready_to_command_ && !tree_runner_.have_ongoing_tree()) {
        tree_runner_.Run(tree_loader_.Load("rise_eff"));
      } else {
        RCLCPP_WARN(this->get_logger(), "Not ready to command");
      }
    });
    hotkeys_.go_to_left_storage.OnPress([&] { RCLCPP_INFO(this->get_logger(), "Store to left key pressed"); });
    hotkeys_.go_to_right_storage.OnPress([&] { RCLCPP_INFO(this->get_logger(), "Store to right key pressed"); });

    RCLCPP_INFO(this->get_logger(), "BT executor initialized");
  }

 private:
  std::atomic<bool> ready_to_command_{false};
  AsyncTreeRunner tree_runner_;
  BT::BehaviorTreeFactory bt_factory_;
  TreeLoader tree_loader_{bt_factory_};

  /** <user input subscribers> **/
  rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_sub_{nullptr};
  rclcpp::Subscription<rm2_referee_msgs::msg::RemoteControl>::SharedPtr fpv_link_input_sub_{nullptr};
  /** </user input subscribers> **/

  struct {
    Hotkey rise_eff,          // R
        go_to_left_storage,   // Z
        go_to_right_storage;  // X
  } hotkeys_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
