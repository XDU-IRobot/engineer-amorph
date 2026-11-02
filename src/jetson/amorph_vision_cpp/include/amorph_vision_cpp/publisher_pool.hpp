// 按话题名懒创建并缓存 publisher，省得每个话题都在节点里占一个成员。
#pragma once

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

namespace amorph::vision {

template <typename MessageType>
class PublisherPool {
 public:
  explicit PublisherPool(rclcpp::Node& node, rclcpp::QoS qos = rclcpp::QoS(10)) : node_(node), qos_(qos) {}

  void Publish(const std::string& topic_name, const MessageType& message) {
    auto& publisher = publishers_[topic_name];
    if (!publisher) {
      publisher = node_.create_publisher<MessageType>(topic_name, qos_);
      RCLCPP_INFO(node_.get_logger(), "创建发布器: %s", topic_name.c_str());
    }
    publisher->publish(message);
  }

 private:
  rclcpp::Node& node_;
  rclcpp::QoS qos_;
  std::unordered_map<std::string, typename rclcpp::Publisher<MessageType>::SharedPtr> publishers_;
};

}  // namespace amorph::vision
