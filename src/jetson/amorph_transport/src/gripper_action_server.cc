#include "gripper_action_server.hpp"

#include <thread>

#include "logger.hpp"

namespace amorph {

using namespace text::literals;

GripperActionServer::GripperActionServer(rclcpp::Node* node, std::string server_name)
    : node_(node), server_name_(std::move(server_name)) {
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

  // TODO

  L::I("Gripper command goal executed."_green);
}

}  // namespace amorph
