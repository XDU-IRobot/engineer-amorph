
#include "node.hpp"

#include "logger.hpp"
#include "moveit_helper.hpp"
#include "var_memory.hpp"

namespace amorph {

using namespace text::literals;
using VM = VarMemory;

AmorphMasterNode::AmorphMasterNode(const rclcpp::NodeOptions &options)
    : Node("amorph_master_node", options),
      execute_task_client_{rclcpp_action::create_client<ExecuteTaskSolution>(this, "/execute_task_solution")},
      mtc_tasks_{PickPlaceTask{"pick_place_task"}, PickEnergyUnitTask{this, "pick_energy_unit_task"}} {
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

                     if (!mtc_tasks_.pick_energy_unit_task.Setup()) {
                       planning_in_progress_.store(false);
                       return std::nullopt;
                     }
                     const bool success = mtc_tasks_.pick_energy_unit_task.plan(10);
                     const auto &task = mtc_tasks_.pick_energy_unit_task.task();
                     // task->reset();
                     task->publishAllSolutions(false);
                     planning_in_progress_.store(false);
                     if (success) {
                       const auto &front_solution = *task->solutions().front();
                       moveit_task_constructor_msgs::msg::Solution msg;
                       front_solution.toMsg(msg);
                       pending_solution_ = msg;
                       current_step_ = 0;
                       L::I("Plan stored successfully. Total steps: %zu. Press Ctrl+S to execute step-by-step."_green,
                            pending_solution_.sub_trajectory.size());
                       return task->solutions().front();
                     }
                     return std::nullopt;
                     // std::this_thread::sleep_for(1s);
                     // L::I("Executing pick and place task..."_greenbg);
                     // task->execute(*best_solution);
                   }).detach();
                 })
      .OnKeyDown(RemoteControl::KEY_S, RemoteControl::KEY_LCTRL, [&](const rm::modules::KeyEvent &) {
        L::I("Key Ctrl+S down"_green);
        if (pending_solution_.sub_trajectory.empty()) {
          L::W("No plan available. Press Ctrl+A first."_yellow);
          return;
        }
        if (current_step_ >= pending_solution_.sub_trajectory.size()) {
          L::I("All steps completed for current plan."_green);
          return;
        }
        if (executing_in_progress_.load()) {
          L::W("Execution in progress, ignored."_yellow);
          return;
        }

        std::jthread([&]() {
          executing_in_progress_.store(true);

          auto goal = ExecuteTaskSolution::Goal();
          goal.solution.sub_trajectory.push_back(pending_solution_.sub_trajectory[current_step_]);
          // Preserve start_scene from the entire solution
          goal.solution.start_scene = pending_solution_.start_scene;

          L::I("Executing step %zu / %zu"_green, current_step_ + 1, pending_solution_.sub_trajectory.size());

          auto send_goal_options = rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions();

          auto goal_handle_future = execute_task_client_->async_send_goal(goal, send_goal_options);
          if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            L::E("Action server not available or rejected goal."_red);
            executing_in_progress_.store(false);
            return;
          }

          auto goal_handle = goal_handle_future.get();
          if (!goal_handle) {
            L::E("Goal was rejected by server"_red);
            executing_in_progress_.store(false);
            return;
          }

          auto result_future = execute_task_client_->async_get_result(goal_handle);
          auto result = result_future.get();

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            L::I("Step %zu executed successfully."_green, current_step_ + 1);
            current_step_++;
            if (current_step_ >= pending_solution_.sub_trajectory.size()) {
              L::I("All steps of the current plan completed. Plan consumed."_green);
              pending_solution_ = moveit_task_constructor_msgs::msg::Solution();
              current_step_ = 0;
            }
          } else {
            L::E("Execution failed for step %zu."_red, current_step_ + 1);
          }

          executing_in_progress_.store(false);
        }).detach();
      })
      .OnKeyDown(RemoteControl::KEY_D, RemoteControl::KEY_LCTRL, [&](const rm::modules::KeyEvent &) {
        L::I("Key Ctrl+D down"_green);
        if (pending_solution_.sub_trajectory.empty()) {
          L::W("No plan available. Press Ctrl+A first."_yellow);
          return;
        }
        if (executing_in_progress_.load()) {
          L::W("Execution in progress, ignored."_yellow);
          return;
        }

        std::jthread([&]() {
          executing_in_progress_.store(true);

          auto goal = ExecuteTaskSolution::Goal();
          goal.solution = pending_solution_;

          L::I("Executing entire plan..."_green);

          auto send_goal_options = rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions();

          auto goal_handle_future = execute_task_client_->async_send_goal(goal, send_goal_options);
          if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            L::E("Action server not available or rejected goal."_red);
            executing_in_progress_.store(false);
            return;
          }

          auto goal_handle = goal_handle_future.get();
          if (!goal_handle) {
            L::E("Goal was rejected by server"_red);
            executing_in_progress_.store(false);
            return;
          }

          auto result_future = execute_task_client_->async_get_result(goal_handle);
          auto result = result_future.get();

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            L::I("Entire plan executed successfully."_green);
            pending_solution_ = moveit_task_constructor_msgs::msg::Solution();
            current_step_ = 0;
          } else {
            L::E("Execution failed."_red);
          }

          executing_in_progress_.store(false);
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
