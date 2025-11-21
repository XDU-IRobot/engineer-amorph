#pragma once

#include <rclcpp/rclcpp.hpp>

#include "sbus_interface/msg/sbus.hpp"
#include "rm2_referee_msgs/msg/remote_control.hpp"
#include "rm2_referee_msgs/msg/custom_robot_data.hpp"

#include "amorph_host/rpc/rpc.hpp"
#include "amorph_host/logger.hpp"
#include "amorph_host/ros_params.hpp"
#include "amorph_host/ros_params_manager.hpp"
#include "amorph_host/mtc/moveit_helper.hpp"
#include "amorph_host/var_memory.hpp"

#include "pick_place_task/pick_place_task.hpp"

namespace amorph {

using namespace std::chrono_literals;
using namespace text::literals;
using VM = VarMemory;

class AmorphArmNode {
 public:
  AmorphArmNode()
      : main_ros_node_{std::make_shared<rclcpp::Node>("amorph_host_node",                     //
                                                      rclcpp::NodeOptions()                   //
                                                          .allow_undeclared_parameters(true)  //
                                                          .automatically_declare_parameters_from_overrides(true))},
        mtc_ros_node_{std::make_shared<rclcpp::Node>("amorph_host_mtc_node",                 //
                                                     rclcpp::NodeOptions()                   //
                                                         .allow_undeclared_parameters(true)  //
                                                         .automatically_declare_parameters_from_overrides(true))} {
    // 临时创建一个节点用来获取参数，避免污染其他两个主程序节点的命名空间
    // TODO: 这样搞不行，明天修一下
    auto param_getter_node = std::make_shared<rclcpp::Node>("param_getter_node",                    //
                                                            rclcpp::NodeOptions()                   //
                                                                .allow_undeclared_parameters(true)  //
                                                                .automatically_declare_parameters_from_overrides(true));
    params_manager_ = std::make_unique<RosParamsManager<RosParams>>(param_getter_node);
    params_manager_->Init();
    ros_params_ = params_manager_->data();
    param_getter_node.reset();
    L::I("Node parameters loaded."_green);

    MoveitHelper::GetInstance().Init(mtc_ros_node_);
    // InitRpc();
    InitUserInput();
    L::I("node initialized."_green);

    std::thread([this] {
      while (!executor_running_.load()) {
        std::this_thread::sleep_for(100ms);
      }
      const auto &task = mtc_tasks_.pick_place_task.task();
      // task->reset();
      mtc_tasks_.pick_place_task.Init(mtc_ros_node_);
      mtc_tasks_.pick_place_task.Plan(10);
      task->publishAllSolutions(false);
      auto best_solution = task->solutions().front();
    }).detach();
  }

  ~AmorphArmNode() {
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
  void InitRpc() {
    static rm::hal::Can rpc_can{"can0"};
    auto fdcan_transport = std::make_shared<FdCanTransport>(rpc_can,                         //
                                                            ros_params_.rpc.miso_frame_id,   //
                                                            ros_params_.rpc.mosi_frame_id);  //
    stm32_.AttachToConnection(fdcan_transport);
    rpc_can.Begin();
    stm32_.rpc()->subscribe("log_message", [this](const LogMessage &msg) -> void {
      switch (msg.log_level) {
        case LogMessage::LogLevel::kDebug:
          L::D("[stm32]"_bold + " %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kInfo:
          L::I("[stm32]"_bold + " %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kWarn:
          L::W("[stm32]"_bold + " %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kError:
          L::E("[stm32]"_bold + " %s", msg.str.c_str());
          break;
        case LogMessage::LogLevel::kFatal:
          L::F("[stm32]"_bold + " %s", msg.str.c_str());
          break;
        default:
          L::I("[stm32] [UNKNOWN LEVEL]"_bold + " %s", msg.str.c_str());
          break;
      }
    });
    timers_.rpc_heartbeat = main_ros_node_->create_wall_timer(300ms, [this] { stm32_.KeepConnection(); });
    L::I("RPC with stm32 ok."_green);
  }

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

  std::unique_ptr<RosParamsManager<RosParams>> params_manager_;
  RosParams ros_params_;

  RPC stm32_;

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
  } mtc_tasks_{};
};

}  // namespace amorph