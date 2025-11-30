#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sbus_interface/msg/sbus.hpp"
#include "rm2_referee_msgs/msg/remote_control.hpp"
#include "rm2_referee_msgs/msg/custom_robot_data.hpp"

#include "logger.hpp"
#include "moveit_helper.hpp"
#include "var_memory.hpp"

#include "pick_place_task/pick_place_task.hpp"
#include "pick_energy_unit_task/pick_energy_unit_task.hpp"

namespace amorph {

using namespace std::chrono_literals;
using namespace text::literals;
using VM = VarMemory;

class AmorphMasterNode {
 public:
  AmorphMasterNode()
      : main_ros_node_{std::make_shared<rclcpp::Node>("amorph_host_node",                     //
                                                      rclcpp::NodeOptions()                   //
                                                          .allow_undeclared_parameters(true)  //
                                                          .automatically_declare_parameters_from_overrides(true))},
        mtc_ros_node_{std::make_shared<rclcpp::Node>("amorph_host_mtc_node",                 //
                                                     rclcpp::NodeOptions()                   //
                                                         .allow_undeclared_parameters(true)  //
                                                         .automatically_declare_parameters_from_overrides(true))} {
    MoveitHelper::GetInstance().Init(mtc_ros_node_);
    InitUserInput();
    L::I("node initialized."_green);

    std::thread([this] {
      while (!executor_running_.load()) {
        std::this_thread::sleep_for(100ms);
      }
      const auto &task = mtc_tasks_.pick_energy_unit_task.task();
      // task->reset();
      mtc_tasks_.pick_energy_unit_task.init(mtc_ros_node_);
      mtc_tasks_.pick_energy_unit_task.plan(10);
      task->publishAllSolutions(false);
      if (task->numSolutions() > 0) {
        auto best_solution = task->solutions().front();
      }

      // std::this_thread::sleep_for(1s);
      // L::I("Executing pick and place task..."_greenbg);
      // task->execute(*best_solution);
    }).detach();
  }

  ~AmorphMasterNode() {
    node_executor_.cancel();
    executor_running_.store(false);
    L::I("node is being destroyed."_green);
  }

  void Spin() {
    node_executor_.add_node(main_ros_node_);
    node_executor_.add_node(mtc_ros_node_);
    executor_running_.store(true);
    node_executor_.spin();
    executor_running_.store(false);
  }

 protected:
  /**
   * @brief 开始接收所有链路的用户输入：sbus遥控器、图传链路键鼠、自定义控制器
   */
  void InitUserInput() {
    subs_.sbus_remote = main_ros_node_->create_subscription<sbus_interface::msg::Sbus>(
        "/sbus", rclcpp::SensorDataQoS(), [this](sbus_interface::msg::Sbus::SharedPtr msg) {
          if (msg->failsafe && !VM::last(msg->failsafe)) {
            L::W("Sbus failsafe!"_yellow);
            return;
          }
        });
    subs_.vt_remote = main_ros_node_->create_subscription<rm2_referee_msgs::msg::RemoteControl>(
        "/rm2_referee/remote_control", rclcpp::SensorDataQoS(),
        [this](rm2_referee_msgs::msg::RemoteControl::SharedPtr msg) {
          static uint16_t last_keyboard_value = 0;
#define KEY_DOWN(key_value) ((msg->keyboard_value == (key_value)) && (msg->keyboard_value ^ last_keyboard_value))
#define KEY_UP(key_value) \
  (((msg->keyboard_value ^ last_keyboard_value) == (key_value)) && (last_keyboard_value == (key_value)))
#define KEY_HOLD(key_value) (msg->keyboard_value == (key_value))
#define KEY_NOT_HOLD(key_value) (msg->keyboard_value != (key_value))
          using RC = rm2_referee_msgs::msg::RemoteControl;
          if (KEY_DOWN(RC::KEY_W)) {
            L::I("Key W down");
          }
          if (KEY_UP(RC::KEY_W)) {
            L::I("Key W up");
          }
          if (KEY_DOWN(RC::KEY_E | RC::KEY_LCTRL)) {
            L::I("Key E + LCTRL down: start pick and place task");
          }
#undef KEY_DOWN
#undef KEY_UP
#undef KEY_HOLD
#undef KEY_NOT_HOLD
          last_keyboard_value = msg->keyboard_value;
        });
    subs_.custom_controller = main_ros_node_->create_subscription<rm2_referee_msgs::msg::CustomRobotData>(
        "/rm2_referee/custom_robot_data", rclcpp::SensorDataQoS(),
        [this](rm2_referee_msgs::msg::CustomRobotData::SharedPtr msg) {

        });
    L::I("User input ok."_green);
  }

  rclcpp::Node::SharedPtr main_ros_node_;
  rclcpp::Node::SharedPtr mtc_ros_node_;
  rclcpp::executors::MultiThreadedExecutor node_executor_;
  std::atomic<bool> executor_running_{false};

  struct {
    rclcpp::TimerBase::SharedPtr rpc_heartbeat{nullptr};
  } timers_{};
  struct {
    rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_remote{nullptr};
    rclcpp::Subscription<rm2_referee_msgs::msg::RemoteControl>::SharedPtr vt_remote{nullptr};
    rclcpp::Subscription<rm2_referee_msgs::msg::CustomRobotData>::SharedPtr custom_controller{nullptr};
  } subs_{};
  struct {
    PickPlaceTask pick_place_task{"pick_place_task"};
    PickEnergyUnitTask pick_energy_unit_task{"pick_energy_unit_task"};
  } mtc_tasks_{};
};

}  // namespace amorph