
#ifndef COMMON_ACTION_SERVER_HPP
#define COMMON_ACTION_SERVER_HPP

#include <rclcpp_action/rclcpp_action.hpp>

template <typename ActionType>
class ActionServer {
 public:
  using GoalHandleType = rclcpp_action::ServerGoalHandle<ActionType>;
  using HandleGoalCallback = std::function<rclcpp_action::GoalResponse(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionType::Goal>)>;
  using HandleCancelCallback = std::function<rclcpp_action::CancelResponse(const std::shared_ptr<GoalHandleType> &)>;
  using HandleAcceptedCallback = std::function<void(const std::shared_ptr<GoalHandleType> &)>;

  ActionServer() = default;
  ActionServer(HandleGoalCallback goal_callback,      //
               HandleCancelCallback cancel_callback,  //
               HandleAcceptedCallback accepted_callback)
      : goal_callback(std::move(goal_callback)),
        cancel_callback(std::move(cancel_callback)),
        accepted_callback(std::move(accepted_callback)) {}

  void Create(rclcpp::Node *node, const std::string &action_name) {
    if (goal_callback == nullptr || cancel_callback == nullptr || accepted_callback == nullptr) {
      throw std::runtime_error("ActionServer callbacks must be set before creating the server.");
    }
    action_server =
        rclcpp_action::create_server<ActionType>(node, action_name, goal_callback, cancel_callback, accepted_callback);
  }

  typename rclcpp_action::Server<ActionType>::SharedPtr action_server{nullptr};
  HandleGoalCallback goal_callback{nullptr};
  HandleCancelCallback cancel_callback{nullptr};
  HandleAcceptedCallback accepted_callback{nullptr};
};

#endif  // COMMON_ACTION_SERVER_HPP