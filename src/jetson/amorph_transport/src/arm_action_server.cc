#include "arm_action_server.hpp"

#include <thread>

#include "logger.hpp"

namespace amorph {

using namespace text::literals;

ArmActionServer::ArmActionServer(rclcpp::Node* node,
                                 std::string server_name,
                                 std::vector<std::string> joint_names,
                                 Stm32& stm32,
                                 TrajectoryTracker<6>& trajectory_tracker)
    : node_(node),
      server_name_(std::move(server_name)),
      joint_names_(std::move(joint_names)),
      stm32_(stm32),
      trajectory_tracker_(trajectory_tracker) {
  server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node_,
      server_name_,
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
        L::I("Received arm trajectory goal");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        L::I("Received arm trajectory cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{[this](std::shared_ptr<GoalHandle> gh) { Execute(gh); }, goal_handle}.detach();
      });
}

void ArmActionServer::Execute(std::shared_ptr<GoalHandle> goal_handle) {
  L::I("Executing arm trajectory goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->joint_names = joint_names_;

  const auto& trajectory = goal->trajectory;

  // 检查 STM32 当前模式是否允许接管
  Stm32FeedbackCompressed::Mode stm32_mode = stm32_.rx_data().mode;
  if (stm32_mode != Stm32FeedbackCompressed::kIdle && stm32_mode != Stm32FeedbackCompressed::kHostControl) {
    L::W("STM32 not in idle/host-control mode (current: %d), aborting trajectory!", static_cast<int>(stm32_mode));
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "STM32 not ready for host control";
    goal_handle->abort(result);
    return;
  }

  // 将轨迹加载到 tracker（内部会做合法性校验和限位）
  trajectory_tracker_.SwitchReferenceTrajectory(trajectory);

  // 请求 STM32 进入上位机控制模式
  stm32_.tx_data_.take_control = true;

  // 等待轨迹执行完毕或被取消
  rclcpp::Rate feedback_rate(50);  // 50Hz 反馈频率
  while (rclcpp::ok()) {
    // 检查是否收到取消请求
    if (goal_handle->is_canceling()) {
      stm32_.tx_data_.take_control = false;
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "Trajectory canceled by client";
      goal_handle->canceled(result);
      L::I("Arm trajectory goal canceled."_yellow);
      return;
    }

    // 检查 STM32 是否出现故障
    Stm32FeedbackCompressed::Mode current_mode = stm32_.rx_data().mode;
    if (current_mode == Stm32FeedbackCompressed::kFault) {
      stm32_.tx_data_.take_control = false;
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "STM32 entered fault state during execution";
      goal_handle->abort(result);
      L::E("STM32 fault during trajectory execution!");
      return;
    }

    // 构造并发布反馈
    feedback->desired = trajectory_tracker_.current_point();
    {
      const auto& rx = stm32_.rx_data();
      feedback->actual.positions.resize(6);
      feedback->actual.velocities.resize(6);
      for (size_t i = 0; i < 6; ++i) {
        feedback->actual.positions[i] = rx.joint_positions[i];
        feedback->actual.velocities[i] = rx.joint_velocities[i];
      }
    }
    feedback->error.positions.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      feedback->error.positions[i] = feedback->desired.positions[i] - feedback->actual.positions[i];
    }
    goal_handle->publish_feedback(feedback);

    // 检查轨迹是否执行完毕
    if (trajectory_tracker_.finished()) {
      break;
    }

    feedback_rate.sleep();
  }

  // 轨迹执行完毕，释放控制权
  stm32_.tx_data_.take_control = false;
  result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
  goal_handle->succeed(result);
  L::I("Arm trajectory goal executed."_green);
}

}  // namespace amorph
