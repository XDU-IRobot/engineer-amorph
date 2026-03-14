#include "execute_task_solution_server.hpp"

#include <thread>

#include "logger.hpp"

namespace amorph {

using namespace text::literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

ExecuteTaskSolutionServer::ExecuteTaskSolutionServer(rclcpp::Node* node, Stm32& stm32,
                                                     TrajectoryTracker<6>& trajectory_tracker)
    : node_(node), stm32_(stm32), trajectory_tracker_(trajectory_tracker) {
  server_ = rclcpp_action::create_server<ExecuteTaskSolution>(
      node_, "execute_task_solution",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecuteTaskSolution::Goal> goal) {
        L::I("Received execute_task_solution goal");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        L::I("Received execute_task_solution cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{[this](std::shared_ptr<GoalHandle> gh) { Execute(gh); }, goal_handle}.detach();
      });
}

void ExecuteTaskSolutionServer::Execute(std::shared_ptr<GoalHandle> goal_handle) {
  L::I("Executing task solution goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteTaskSolution::Result>();
  auto feedback = std::make_shared<ExecuteTaskSolution::Feedback>();

  const auto& solution = goal->solution;
  L::I("Task solution contains %zu sub-trajectories", solution.sub_trajectory.size());

  // 遍历所有子轨迹
  for (size_t i = 0; i < solution.sub_trajectory.size(); ++i) {
    if (goal_handle->is_canceling()) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
      L::I("Task solution execution canceled."_yellow);
      return;
    }

    const auto& sub_traj = solution.sub_trajectory[i];
    const auto& exec_info = sub_traj.execution_info;
    const auto& joint_traj = sub_traj.trajectory.joint_trajectory;

    // 获取控制器名称
    std::string controller_name;
    if (!exec_info.controller_names.empty()) {
      controller_name = exec_info.controller_names[0];
    }

    L::I("Executing sub-trajectory %zu/%zu for controller: %s", i + 1, solution.sub_trajectory.size(),
         controller_name.c_str());

    // 跳过空轨迹（如 ModifyPlanningScene 等不产生运动的 stage）
    if (joint_traj.points.empty()) {
      L::I("Empty trajectory, skipping");
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
      continue;
    }

    // 根据 controller 名称判断是机械臂还是夹爪
    if (controller_name.find("arm") != std::string::npos ||
        controller_name.find("follow_joint_trajectory") != std::string::npos) {
      // 这是机械臂轨迹，检查关节数量是否匹配
      if (!joint_traj.joint_names.empty() && joint_traj.joint_names.size() != 6) {
        L::W("Arm trajectory has %zu joints, expected 6, skipping", joint_traj.joint_names.size());
        feedback->sub_id = static_cast<uint32_t>(i);
        feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
        goal_handle->publish_feedback(feedback);
        continue;
      }

      auto arm_goal = std::make_shared<FollowJointTrajectory::Goal>();
      arm_goal->trajectory = joint_traj;

      // 检查 STM32 状态
      Stm32FeedbackCompressed::Mode stm32_mode = stm32_.rx_data().mode;
      if (stm32_mode != Stm32FeedbackCompressed::kIdle && stm32_mode != Stm32FeedbackCompressed::kHostControl) {
        L::W("STM32 not in idle/host-control mode (current: %d), aborting task!", static_cast<int>(stm32_mode));
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
        goal_handle->abort(result);
        return;
      }

      // 加载轨迹
      trajectory_tracker_.SwitchReferenceTrajectory(arm_goal->trajectory);
      stm32_.tx_data_.take_control = true;

      // 等待轨迹执行完成
      rclcpp::Rate rate(50);
      while (rclcpp::ok() && !trajectory_tracker_.finished()) {
        if (goal_handle->is_canceling()) {
          stm32_.tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          goal_handle->canceled(result);
          L::I("Task solution execution canceled."_yellow);
          return;
        }

        Stm32FeedbackCompressed::Mode current_mode = stm32_.rx_data().mode;
        if (current_mode == Stm32FeedbackCompressed::kFault) {
          stm32_.tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
          goal_handle->abort(result);
          L::E("STM32 fault during task execution!");
          return;
        }

        rate.sleep();
      }

      stm32_.tx_data_.take_control = false;

      // 添加延时，确保机械臂停止稳定后再进行下一个动作
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } else if (controller_name.find("gripper") != std::string::npos) {
      // 这是夹爪轨迹
      L::I("Gripper trajectory execution");
      if (joint_traj.points.empty() || joint_traj.points.back().positions.empty()) {
        L::W("Gripper trajectory is empty, skipping");
        continue;
      }

      double target_position = joint_traj.points.back().positions[0];
      L::I("Target gripper position: %f", target_position);

      Stm32FeedbackCompressed::Mode stm32_mode = stm32_.rx_data().mode;
      if (stm32_mode != Stm32FeedbackCompressed::kIdle && stm32_mode != Stm32FeedbackCompressed::kHostControl) {
        L::W("STM32 not in idle/host-control mode (current: %d), aborting task!", static_cast<int>(stm32_mode));
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
        goal_handle->abort(result);
        return;
      }

      stm32_.tx_data_.take_control = true;
      bool target_open = target_position > -0.01;
      stm32_.tx_data_.gripper_open = target_open;

      // 短暂延时，确保 STM32 收到命令并更新一次反馈状态
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      rclcpp::Rate rate(50);
      while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
          stm32_.tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          goal_handle->canceled(result);
          L::I("Task solution execution canceled."_yellow);
          return;
        }

        Stm32FeedbackCompressed::Mode current_mode = stm32_.rx_data().mode;
        if (current_mode == Stm32FeedbackCompressed::kFault) {
          stm32_.tx_data_.take_control = false;
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
          goal_handle->abort(result);
          L::E("STM32 fault during task execution!");
          return;
        }

        bool reached_goal = false;
        if (target_open) {
          reached_goal = (stm32_.rx_data().gripper_state == Stm32FeedbackCompressed::kOpen);
        } else {
          reached_goal = (stm32_.rx_data().gripper_state == Stm32FeedbackCompressed::kClose);
        }

        if (reached_goal) {
          break;
        }

        rate.sleep();
      }

      stm32_.tx_data_.take_control = false;

      // 添加延时，确保夹爪动作完成稳定后再进行下一个动作
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    } else if (controller_name.empty()) {
      // 空控制器名称，可能是 ModifyPlanningScene 等 stage，已在前面过滤空轨迹时处理
      L::I("No controller specified, skipping (likely a planning scene modification)");

      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    } else {
      L::W("Unknown controller type: %s, skipping", controller_name.c_str());

      // 更新反馈
      feedback->sub_id = static_cast<uint32_t>(i);
      feedback->sub_no = static_cast<uint32_t>(solution.sub_trajectory.size());
      goal_handle->publish_feedback(feedback);
    }
  }

  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  goal_handle->succeed(result);
  L::I("Task solution executed successfully."_green);
}

}  // namespace amorph
