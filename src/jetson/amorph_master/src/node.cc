
#include "node.hpp"

#include "logger.hpp"
#include "moveit_helper.hpp"
#include "var_memory.hpp"

namespace amorph {

using namespace text::literals;
using VM = VarMemory;

AmorphMasterNode::AmorphMasterNode(const rclcpp::NodeOptions &options)
    : Node("amorph_master_node", options),
      tf_buffer_{std::make_shared<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)} {
  MoveitHelper::GetInstance().Init(std::shared_ptr<rclcpp::Node>{this});
  InitUserInput();
  L::I("node initialized."_green);
}

AmorphMasterNode::~AmorphMasterNode() { L::I("node is being destroyed."_green); }

void AmorphMasterNode::InitUserInput() {
  subs_.sbus_remote = this->create_subscription<sbus_interface::msg::Sbus>(
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
                     planning_in_progress_.store(true);

                     // 获取 world->energy_unit_0_smoothed 变换
                     geometry_msgs::msg::TransformStamped transform_stamped;
                     try {
                       transform_stamped =
                           tf_buffer_->lookupTransform("world", "energy_unit_0_smoothed", tf2::TimePointZero);
                     } catch (const tf2::TransformException &ex) {
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

                     mtc_tasks_.pick_energy_unit_task.init(this->shared_from_this(), object_pose);
                     const bool success = mtc_tasks_.pick_energy_unit_task.plan(10);
                     const auto &task = mtc_tasks_.pick_energy_unit_task.task();
                     // task->reset();
                     task->publishAllSolutions(false);
                     planning_in_progress_.store(false);
                     if (success) {
                       task->execute(*task->solutions().front());
                       return task->solutions().front();
                     }
                     return std::nullopt;
                     // std::this_thread::sleep_for(1s);
                     // L::I("Executing pick and place task..."_greenbg);
                     // task->execute(*best_solution);
                   }).detach();
                 });
  subs_.vt_remote = this->create_subscription<RemoteControl>(
      "/rm_referee/remote_control", rclcpp::SensorDataQoS(),
      [this](RemoteControl::SharedPtr msg) { keyboard_handler.Update(msg->keyboard_value); });
  L::I("User input ok."_green);
}

}  // namespace amorph

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amorph::AmorphMasterNode)
