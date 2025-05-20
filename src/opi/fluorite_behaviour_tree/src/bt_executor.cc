
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
        [&](const rm2_referee_msgs::msg::RemoteControl::SharedPtr msg) { kh_.Poll(msg->keyboard_value); });

    hotkeys_.retract =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_R};
    hotkeys_.take_silver =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_F};
    hotkeys_.store_to_right =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_C};
    hotkeys_.pick_from_right =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_V};
    hotkeys_.pick_from_right_top =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LSHIFT | rm2_referee_msgs::msg::RemoteControl::KEY_V};
    hotkeys_.store_to_left =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_Z};
    hotkeys_.pick_from_left =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LCTRL | rm2_referee_msgs::msg::RemoteControl::KEY_X};
    hotkeys_.pick_from_left_top =
        Hotkey{rm2_referee_msgs::msg::RemoteControl::KEY_LSHIFT | rm2_referee_msgs::msg::RemoteControl::KEY_X};
    kh_ << hotkeys_.retract << hotkeys_.take_silver << hotkeys_.store_to_right << hotkeys_.pick_from_right << hotkeys_.pick_from_right_top
        << hotkeys_.store_to_left << hotkeys_.pick_from_left << hotkeys_.pick_from_left_top;

    // define keymap
    hotkeys_.retract.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Retract arm!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("retract"));
    });
    hotkeys_.take_silver.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Take silver!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("take_silver"));
    });
    hotkeys_.store_to_right.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Store to right!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("store_to_right"));
    });
    hotkeys_.pick_from_right.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Pick from right!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("pick_from_right"));
    });
    hotkeys_.store_to_left.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Store to left!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("store_to_left"));
    });
    hotkeys_.pick_from_left.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Pick from left!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("pick_from_left"));
    });
    hotkeys_.pick_from_left_top.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Pick from left top!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("pick_from_left_top"));
    });
    hotkeys_.pick_from_right_top.OnPress([&] {
      RCLCPP_INFO(this->get_logger(), "Pick from right top!");
      if (!CheckIsReadyToCommand()) {
        return;
      }
      tree_runner_.Run(tree_loader_.Load("pick_from_right_top"));
    });

    RCLCPP_INFO(this->get_logger(), "BT executor initialized");
  }

 private:
  bool CheckIsReadyToCommand() {
    if (ready_to_command_ && !tree_runner_.have_ongoing_tree()) {
      RCLCPP_INFO(this->get_logger(), "Ready to command");
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "Not ready to command");
    return false;
  }

  std::atomic<bool> ready_to_command_{false};
  AsyncTreeRunner tree_runner_;
  BT::BehaviorTreeFactory bt_factory_;
  TreeLoader tree_loader_{bt_factory_};

  /** <user input subscribers> **/
  rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_sub_{nullptr};
  rclcpp::Subscription<rm2_referee_msgs::msg::RemoteControl>::SharedPtr fpv_link_input_sub_{nullptr};
  /** </user input subscribers> **/

  struct {
    Hotkey retract,           // LCTRL + R
        take_silver,          // LCTRL + F
        store_to_right,       // LCTRL + C
        pick_from_right,      // LCTRL + V
        pick_from_right_top,  // LSHIFT + V
        store_to_left,        // LCTRL + Z
        pick_from_left,       // LCTRL + X
        pick_from_left_top;   // LSHIFT + X
  } hotkeys_;
  KeypressHandler kh_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
