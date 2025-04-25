/**
 * @file  ros_node.hpp
 * @brief 一个rclcpp::Node单例，供决策树节点使用
 */

#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>

class RosNodeSingleton {
 public:
  static RosNodeSingleton &GetInstance() {
    static RosNodeSingleton instance;
    return instance;
  }

  rclcpp::Node::SharedPtr GetNode() { return node_; }

  static void Debug(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    const auto logger = GetInstance().GetNode()->get_logger();
    RCLCPP_DEBUG(logger, msg, args);
    va_end(args);
  }

  static void Info(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    const auto logger = GetInstance().GetNode()->get_logger();
    RCLCPP_INFO(logger, msg, args);
    va_end(args);
  }

  static void Warn(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    const auto logger = GetInstance().GetNode()->get_logger();
    RCLCPP_WARN(logger, msg, args);
    va_end(args);
  }

  static void Error(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    const auto logger = GetInstance().GetNode()->get_logger();
    RCLCPP_ERROR(logger, msg, args);
    va_end(args);
  }

 private:
  RosNodeSingleton()
      : node_(rclcpp::Node::make_shared("fluorite_behaviour_tree")),
        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    executor_->add_node(node_);
    executor_thread_ = std::thread([&] { executor_->spin(); });
  };
  ~RosNodeSingleton() {
    executor_->cancel();
    executor_->remove_node(node_);
    executor_thread_.join();
    node_.reset();
    executor_.reset();
  }

  RosNodeSingleton(const RosNodeSingleton &) = delete;
  RosNodeSingleton &operator=(const RosNodeSingleton &) = delete;
  RosNodeSingleton(RosNodeSingleton &&) = delete;
  RosNodeSingleton &operator=(RosNodeSingleton &&) = delete;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic<bool> in_use_{false};
};

#endif  // ROS_NODE_HPP