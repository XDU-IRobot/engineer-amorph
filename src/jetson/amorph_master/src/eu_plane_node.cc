// 架面拟合节点：接收视觉节点的能量单元检测结果，拟合它们 z 轴所在的平面并发布成 Marker。
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "rack_plane.hpp"
#include "visualization.hpp"

namespace amorph_master {

/// @brief 视觉节点的能量单元检测结果里，各单元的 z 轴一般是共面的（都插在同一个架子上）。
/// 拟合出这个平面，以 MarkerArray 发布供抓取任务参考。
class EuPlaneNode : public rclcpp::Node {
 public:
  explicit EuPlaneNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node{"eu_plane_node", options} {}

  /// @brief 创建可视化模块，每收到一帧检测结果就拟合、发布一次架面。
  void Start() {
    visualization_ = std::make_unique<Visualization>(shared_from_this());
    const std::string topic = declare_parameter("detections_topic", "eu_detections");
    detection_subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        topic, rclcpp::SensorDataQoS(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr &message) { FitAndPublish(*message); });
  }

 private:
  void FitAndPublish(const vision_msgs::msg::Detection3DArray &message) {
    const std::optional<Plane> plane = FitRackPlane(message);
    if (!plane.has_value()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "not enough eu to fit plane (%zu)",
                           message.detections.size());
      return;
    }
    // 直接在消息的坐标系下发布，省一次 tf 变换
    visualization_->PublishPlane(message.header.frame_id, plane->center, plane->normal, kPlaneSize,
                                 {0.2f, 0.8f, 1.0f, 0.4f}, kPlaneTopic);
  }

  static constexpr const char *kPlaneTopic = "rack_plane";
  static constexpr double kPlaneSize = 1.0;  ///< 发布平面的边长，单位米

  std::unique_ptr<Visualization> visualization_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscription_;
};

}  // namespace amorph_master

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amorph_master::EuPlaneNode>();
  node->Start();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
