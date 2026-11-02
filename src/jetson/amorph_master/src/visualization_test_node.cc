// 测试程序：在 world 坐标系 (0, 0, 1) 处发布一个绕 Y 轴旋转的长方形平面，30Hz 刷新。
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "visualization.hpp"

namespace amorph_master {

class VisualizationTestNode : public rclcpp::Node {
 public:
  VisualizationTestNode() : rclcpp::Node{"visualization_test_node"} {}

  void Start() {
    visualization_ = std::make_unique<Visualization>(shared_from_this());
    start_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / kFrequencyHz), [this]() { PublishPlane(); });
  }

 private:
  static constexpr double kFrequencyHz = 60.0;
  static constexpr double kAngularVelocity = 1.0;  ///< 绕 Y 轴的角速度，rad/s
  static inline const Eigen::Vector3d kCenter{0.0, 0.0, 1.0};
  static inline const Eigen::Vector2d kSize{1.0, 0.5};  ///< (width, height)，长方形

  void PublishPlane() {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    const double angle = kAngularVelocity * elapsed;
    // 绕 Y 轴旋转 angle 后，原本的 +Z 法向量变为 (sin, 0, cos)
    visualization_->PublishPlane("world", kCenter, Eigen::Vector3d{std::sin(angle), 0.0, std::cos(angle)}, kSize);
  }

  std::unique_ptr<Visualization> visualization_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace amorph_master

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amorph_master::VisualizationTestNode>();
  node->Start();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
