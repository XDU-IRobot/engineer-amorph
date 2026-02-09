#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <librm.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sbus_interface/msg/sbus.hpp"
#include "rm_referee_msgs/msg/remote_control.hpp"

#include "logger.hpp"
#include "moveit_helper.hpp"
#include "var_memory.hpp"

#include "pick_place_task/pick_place_task.hpp"
#include "pick_energy_unit_task/pick_energy_unit_task.hpp"

namespace amorph {

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
                                                         .automatically_declare_parameters_from_overrides(true))},
        tf_buffer_{std::make_shared<tf2_ros::Buffer>(main_ros_node_->get_clock())},
        tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)} {
    MoveitHelper::GetInstance().Init(mtc_ros_node_);
    InitUserInput();
    L::I("node initialized."_green);
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

    using moveit::task_constructor::SolutionBase;
    using rm_referee_msgs::msg::RemoteControl;
    static rm::modules::KeyboardHandler keyboard_handler(RemoteControl::KEY_LCTRL | RemoteControl::KEY_LSHIFT);

    // 注册键位
    keyboard_handler                                                //
        .OnKeyDown(RemoteControl::KEY_A, RemoteControl::KEY_LCTRL,  //
                   [&](const rm::modules::KeyEvent &) {
                     L::I("Key Ctrl+A down"_green);
                     if (planning_in_progress_.load()) {
                       L::W("Planning in progress, canceled."_yellow);
                       return;
                     }
                     std::jthread([&]() -> std::optional<const std::shared_ptr<const SolutionBase>> {
                       if (!executor_running_.load()) {
                         return std::nullopt;
                       }
                       planning_in_progress_.store(true);
                       
                       // 获取 world->energy_unit_0_smoothed 变换
                       geometry_msgs::msg::TransformStamped transform_stamped;
                       try {
                         transform_stamped = tf_buffer_->lookupTransform(
                             "world", "energy_unit_0_smoothed", tf2::TimePointZero);
                       } catch (const tf2::TransformException& ex) {
                         L::E("Failed to get transform world->energy_unit_0_smoothed: %s", ex.what());
                         planning_in_progress_.store(false);
                         return std::nullopt;
                       }
                       
                       // 将 Transform 转换为 Pose
                       geometry_msgs::msg::Pose object_pose;
                       object_pose.position.x = transform_stamped.transform.translation.x;
                       object_pose.position.y = transform_stamped.transform.translation.y;
                       object_pose.position.z = transform_stamped.transform.translation.z;
                       object_pose.orientation = transform_stamped.transform.rotation;
                       
                       mtc_tasks_.pick_energy_unit_task.init(mtc_ros_node_, object_pose);
                       mtc_tasks_.pick_energy_unit_task.plan(10);
                       const auto &task = mtc_tasks_.pick_energy_unit_task.task();
                       // task->reset();
                       task->publishAllSolutions(false);
                       planning_in_progress_.store(false);
                       if (task->numSolutions() > 0) {
                        task->execute(*task->solutions().front());
                         return task->solutions().front();
                       }
                       return std::nullopt;
                       // std::this_thread::sleep_for(1s);
                       // L::I("Executing pick and place task..."_greenbg);
                       // task->execute(*best_solution);
                     }).detach();
                   });
    subs_.vt_remote = main_ros_node_->create_subscription<RemoteControl>(
        "/rm_referee/remote_control", rclcpp::SensorDataQoS(),
        [this](RemoteControl::SharedPtr msg) { keyboard_handler.Update(msg->keyboard_value); });
    L::I("User input ok."_green);
  }

  rclcpp::Node::SharedPtr main_ros_node_;
  rclcpp::Node::SharedPtr mtc_ros_node_;
  rclcpp::executors::MultiThreadedExecutor node_executor_;
  std::atomic<bool> executor_running_{false};
  std::atomic<bool> planning_in_progress_{false};
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  struct {
    rclcpp::TimerBase::SharedPtr rpc_heartbeat{nullptr};
  } timers_{};
  struct {
    rclcpp::Subscription<sbus_interface::msg::Sbus>::SharedPtr sbus_remote{nullptr};
    rclcpp::Subscription<rm_referee_msgs::msg::RemoteControl>::SharedPtr vt_remote{nullptr};
  } subs_{};
  struct {
    PickPlaceTask pick_place_task{"pick_place_task"};
    PickEnergyUnitTask pick_energy_unit_task{"pick_energy_unit_task"};
  } mtc_tasks_{};
};

}  // namespace amorph