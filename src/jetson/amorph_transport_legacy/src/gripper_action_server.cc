#include "gripper_action_server.hpp"

#include <thread>

#include "logger.hpp"

namespace amorph {

using namespace text::literals;

GripperActionServer::GripperActionServer(rclcpp::Node* node, std::string server_name, Stm32& stm32)
    : node_(node), server_name_(std::move(server_name)), stm32_(stm32) {
  server_ = rclcpp_action::create_server<GripperCommand>(
      node_,
      server_name_,
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal) {
        L::I("Received gripper command goal");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        L::I("Received gripper command cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{[this](std::shared_ptr<GoalHandle> gh) { Execute(gh); }, goal_handle}.detach();
      });
}

void GripperActionServer::Execute(std::shared_ptr<GoalHandle> goal_handle) {
  L::I("Executing gripper command goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GripperCommand::Result>();
  L::I("Gripper command: position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);

  // 检查 STM32 当前模式是否允许接管
  Stm32FeedbackCompressed::Mode stm32_mode = stm32_.rx_data().mode;
  if (stm32_mode != Stm32FeedbackCompressed::kIdle && stm32_mode != Stm32FeedbackCompressed::kHostControl) {
    L::W("STM32 not in idle/host-control mode (current: %d), aborting gripper command!", static_cast<int>(stm32_mode));
    result->stalled = true;
    goal_handle->abort(result);
    return;
  }

  // 请求 STM32 进入上位机控制模式并设置夹爪状态
  stm32_.tx_data_.take_control = true;
  bool target_open = goal->command.position > -0.01;
  stm32_.tx_data_.gripper_open = target_open;

  // 短暂延时，确保 STM32 有足够的时间响应我们的命令并更新其反馈状态，避免读到旧状态而直接返回
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  rclcpp::Rate feedback_rate(50);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stm32_.tx_data_.take_control = false;
      goal_handle->canceled(result);
      L::I("Gripper command goal canceled."_yellow);
      return;
    }

    Stm32FeedbackCompressed::Mode current_mode = stm32_.rx_data().mode;
    if (current_mode == Stm32FeedbackCompressed::kFault) {
      stm32_.tx_data_.take_control = false;
      goal_handle->abort(result);
      L::E("STM32 fault during gripper execution!");
      return;
    }

    auto feedback = std::make_shared<GripperCommand::Feedback>();
    switch (stm32_.rx_data().gripper_state) {
      case Stm32FeedbackCompressed::kOpen:
        feedback->position = 0.08;
        break;
      case Stm32FeedbackCompressed::kClose:
        feedback->position = 0.0;
        break;
      case Stm32FeedbackCompressed::kOpening:
      case Stm32FeedbackCompressed::kClosing:
        feedback->position = 0.04;
        break;
    }

    if (target_open) {
      // 期待打开的动作，必须明确达到 kOpen 状态才算成功
      feedback->reached_goal = (stm32_.rx_data().gripper_state == Stm32FeedbackCompressed::kOpen);
    } else {
      // 期待关闭的动作，当夹爪夹到物体时可能持续处于 kClosing 而无法达到完全的 kClose 状态
      // 这里根据实际情况决定是否也把 kClosing 当作一定程度的目标完成 (如果你要求必须夹紧才算，可以仅保留kClose)
      feedback->reached_goal = (stm32_.rx_data().gripper_state == Stm32FeedbackCompressed::kClose);
    }

    goal_handle->publish_feedback(feedback);

    if (feedback->reached_goal) {
      result->position = feedback->position;
      result->reached_goal = true;
      break;
    }

    feedback_rate.sleep();
  }

  stm32_.tx_data_.take_control = false;
  goal_handle->succeed(result);

  L::I("Gripper command goal executed."_green);
}

}  // namespace amorph
