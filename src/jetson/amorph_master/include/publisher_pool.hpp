#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>

namespace amorph_master {

template <typename MessageType>
class PublisherPool {
 public:
  using SharedPtr = std::shared_ptr<PublisherPool<MessageType>>;

  explicit PublisherPool(rclcpp::Node::SharedPtr node, rclcpp::QoS qos = rclcpp::SensorDataQoS())
      : node_(std::move(node)), qos_(qos) {}
  void Publish(const std::string &topic_name, const typename MessageType::SharedPtr &&message) {
    if (bypassed_.load()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (publishers_.find(topic_name) == publishers_.end()) {
        // 如果不存在该topic的publisher，则创建一个新的
        RCLCPP_INFO(node_->get_logger(), "Creating publisher for topic: %s", topic_name.c_str());
        publishers_[topic_name] = node_->create_publisher<MessageType>(topic_name, qos_);
      }
      // 发布消息
      publishers_[topic_name]->publish(*message);
    }
  }

  void Bypass(bool bypass) { bypassed_.store(bypass); }

 private:
  std::atomic<bool> bypassed_{false};
  std::mutex mtx_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::QoS qos_;
  std::unordered_map<std::string,                                         ///< topic name
                     typename rclcpp::Publisher<MessageType>::SharedPtr>  ///< publisher
      publishers_;
};

}  // namespace amorph_master