
#ifndef COMMON_ACTION_CLIENT_HPP
#define COMMON_ACTION_CLIENT_HPP

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

template <typename ActionType>
class ActionClient {
 public:
  using GoalHandleType = rclcpp_action::ClientGoalHandle<ActionType>;
  using GoalResponseCallback = std::function<void(const typename GoalHandleType::SharedPtr &)>;
  using FeedbackCallback = std::function<void(typename GoalHandleType::SharedPtr,
                                              const std::shared_ptr<const typename ActionType::Feedback>)>;
  using ResultCallback = std::function<void(const typename GoalHandleType::WrappedResult &)>;

  ActionClient() = default;
  ActionClient(GoalResponseCallback goal_callback,  //
               FeedbackCallback cancel_callback,    //
               ResultCallback accepted_callback)
      : goal_callback(std::move(goal_callback)),
        cancel_callback(std::move(cancel_callback)),
        accepted_callback(std::move(accepted_callback)) {}

  void Create(rclcpp::Node::SharedPtr node, const std::string &action_name) {
    action_client = rclcpp_action::create_client<ActionType>(node, action_name);
    action_name_ = action_name;
    node_ = node;
  }
  bool CheckServerOnline() {
    if (!action_client) {
      RCLCPP_ERROR(node_->get_logger(), "Action client not created yet!");
      return false;
    }
    if (!action_client->wait_for_action_server()) {
      RCLCPP_ERROR(node_->get_logger(), "Action server '%s' is not available", action_name_.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Action server '%s' is available", action_name_.c_str());
    return true;
  }

  void AsyncSendGoal(typename ActionType::Goal::SharedPtr goal,    //
                     GoalResponseCallback goal_response_callback,  //
                     FeedbackCallback feedback_callback,           //
                     ResultCallback result_callback) {
    goal_response_callback = std::move(goal_response_callback);
    feedback_callback = std::move(feedback_callback);
    result_callback = std::move(result_callback);
    const auto send_goal_options = [&] {
      typename rclcpp_action::Client<ActionType>::SendGoalOptions options;
      options.goal_response_callback = goal_response_callback;
      options.feedback_callback = feedback_callback;
      options.result_callback = result_callback;
      return options;
    }();
    action_client->async_send_goal(*goal, send_goal_options);
  }

  std::string action_name_;
  rclcpp::Node::SharedPtr node_{nullptr};
  typename rclcpp_action::Client<ActionType>::SharedPtr action_client{nullptr};
  GoalResponseCallback goal_callback{nullptr};
  FeedbackCallback cancel_callback{nullptr};
  ResultCallback accepted_callback{nullptr};
};

#endif  // COMMON_ACTION_CLIENT_HPP
