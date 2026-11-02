#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "publisher_pool.hpp"

namespace amorph_master {

class Visualization {
 public:
  using MarkerArrayPublisherPool = PublisherPool<visualization_msgs::msg::MarkerArray>;
  using TrajectoryPublisherPool = PublisherPool<moveit_msgs::msg::DisplayTrajectory>;
  using RobotStatePublisherPool = PublisherPool<moveit_msgs::msg::DisplayRobotState>;

  explicit Visualization(const rclcpp::Node::SharedPtr &node)
      : publisher_pool_(std::make_shared<MarkerArrayPublisherPool>(node)),
        // RViz 的显示插件以 reliable 订阅，PublisherPool 默认的 SensorDataQoS 是 best effort，会被丢弃
        trajectory_publisher_pool_(std::make_shared<TrajectoryPublisherPool>(node, rclcpp::ServicesQoS())),
        robot_state_publisher_pool_(std::make_shared<RobotStatePublisherPool>(node, rclcpp::QoS(1))) {}

  /// @brief 发布一个平面及其法向箭头 (visualization_msgs/MarkerArray)
  /// @param frame_id 平面所在坐标系，如 "camera_depth_optical_frame"
  /// @param origin 平面中心点，单位米
  /// @param normal 平面法向量，无需归一化，决定平面朝向与箭头指向
  /// @param size 平面边长 (width, height)，两者相等即正方形
  /// @param color RGBA 颜色，alpha < 1 时为半透明
  /// @param topic 发布的话题名
  /// @param marker_id Marker 的 id，用于区分同一话题下的多个平面
  /// @param header 可选的 ROS2 消息头 (使用其中的时间戳)；为空时取当前 frame_id
  /// @param double_sided 是否双面可见 (RViz2 的 TRIANGLE_LIST 受光照/法线影响)
  /// @return 发布出去的 MarkerArray，第 0 个是平面，第 1 个是法向箭头
  visualization_msgs::msg::MarkerArray PublishPlane(
      const std::string &frame_id, const Eigen::Vector3d &origin, const Eigen::Vector3d &normal,
      const Eigen::Vector2d &size, const Eigen::Vector4f &color = {0.2f, 0.8f, 1.0f, 0.5f},
      const std::string &topic = "plane", int marker_id = 0,
      const std::optional<std_msgs::msg::Header> &header = std::nullopt, bool double_sided = true) const {
    const double width = size.x();
    const double height = size.y();

    std_msgs::msg::Header header_msg;
    if (header.has_value()) {
      header_msg = header.value();
    } else {
      header_msg.frame_id = frame_id;
    }

    // 平面与箭头共用的姿态：局部 +Z 对齐到法向
    const Eigen::Quaterniond quaternion =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal.normalized());

    visualization_msgs::msg::Marker marker;
    marker.header = header_msg;
    marker.header.frame_id = frame_id;
    marker.ns = "plane";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置放在平面中心，姿态由法向量决定 (TRIANGLE_LIST 的 scale 不缩放顶点)
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.color.a = color.w();

    // 局部坐标下以原点为中心的四个角 (逆时针)
    const double half_width = width / 2.0;
    const double half_height = height / 2.0;
    const std::array<Eigen::Vector3d, 4> corners = {
        Eigen::Vector3d{-half_width, -half_height, 0.0},
        Eigen::Vector3d{half_width, -half_height, 0.0},
        Eigen::Vector3d{half_width, half_height, 0.0},
        Eigen::Vector3d{-half_width, half_height, 0.0}};
    const auto make_point = [&corners](int index) {
      geometry_msgs::msg::Point point;
      point.x = corners[index].x();
      point.y = corners[index].y();
      point.z = corners[index].z();
      return point;
    };

    // 两个三角形拼成矩形
    marker.points.reserve(double_sided ? 12 : 6);
    for (const int index : {0, 1, 2, 0, 2, 3}) {
      marker.points.push_back(make_point(index));
    }
    // 反向再画一遍，避免背面因法线朝外而发黑
    if (double_sided) {
      for (const int index : {0, 2, 1, 0, 3, 2}) {
        marker.points.push_back(make_point(index));
      }
    }

    marker.lifetime = builtin_interfaces::msg::Duration();  // 0 = 永久

    // 从平面中心沿法向伸出的箭头，长度取平面短边的 60%
    visualization_msgs::msg::Marker arrow = marker;
    arrow.ns = "plane_normal";
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.points.clear();
    // ARROW 沿局部 +X 延伸，所以要把 +X 转到法向，不能沿用平面那个把 +Z 转到法向的姿态
    const Eigen::Quaterniond arrow_quaternion =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), normal.normalized());
    arrow.pose.orientation.x = arrow_quaternion.x();
    arrow.pose.orientation.y = arrow_quaternion.y();
    arrow.pose.orientation.z = arrow_quaternion.z();
    arrow.pose.orientation.w = arrow_quaternion.w();
    arrow.scale.x = 0.6 * size.minCoeff();  // 长度
    arrow.scale.y = 0.02;                   // 杆直径
    arrow.scale.z = 0.04;                   // 箭头直径
    arrow.color.a = 1.0;                    // 箭头保持实心，避免半透明看不清指向

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    markers.markers.push_back(std::move(arrow));
    publisher_pool_->Publish(topic, std::make_shared<visualization_msgs::msg::MarkerArray>(markers));
    return markers;
  }

  /// @brief 正方形平面的便捷重载，边长标量
  visualization_msgs::msg::MarkerArray PublishPlane(
      const std::string &frame_id, const Eigen::Vector3d &origin, const Eigen::Vector3d &normal, double size,
      const Eigen::Vector4f &color = {0.2f, 0.8f, 1.0f, 0.5f}, const std::string &topic = "plane", int marker_id = 0,
      const std::optional<std_msgs::msg::Header> &header = std::nullopt, bool double_sided = true) const {
    return PublishPlane(frame_id, origin, normal, Eigen::Vector2d{size, size}, color, topic, marker_id, header,
                        double_sided);
  }

  /// @brief 发布一个坐标系标记 (visualization_msgs/Marker, LINE_LIST)，三根轴分别为红绿蓝
  /// @param frame_id 位姿所在坐标系
  /// @param pose 坐标系位姿，姿态的 X/Y/Z 列即三根轴的方向
  /// @param length 三根轴的长度，单位米
  /// @param topic 发布的话题名
  /// @param marker_id Marker 的 id，用于区分同一话题下的多个坐标系
  /// @param header 可选的 ROS2 消息头 (使用其中的时间戳)；为空时取当前 frame_id
  /// @return 发布出去的 Marker
  visualization_msgs::msg::Marker PublishFrame(const std::string &frame_id, const Eigen::Isometry3d &pose,
                                               double length = 0.1, const std::string &topic = "frame",
                                               int marker_id = 0,
                                               const std::optional<std_msgs::msg::Header> &header = std::nullopt) const {
    std_msgs::msg::Header header_msg;
    if (header.has_value()) {
      header_msg = header.value();
    } else {
      header_msg.frame_id = frame_id;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = header_msg;
    marker.header.frame_id = frame_id;
    marker.ns = "frame";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond quaternion{pose.linear()};
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    // LINE_LIST 逐点使用 marker.color，所以把颜色写进每个点上；scale.x 为线宽
    marker.scale.x = 0.01;
    const auto add_axis = [&](int axis, const Eigen::Vector4f &color) {
      geometry_msgs::msg::Point origin;
      geometry_msgs::msg::Point tip;
      tip.x = length * (axis == 0 ? 1.0 : 0.0);
      tip.y = length * (axis == 1 ? 1.0 : 0.0);
      tip.z = length * (axis == 2 ? 1.0 : 0.0);
      std_msgs::msg::ColorRGBA rgba;
      rgba.r = color.x();
      rgba.g = color.y();
      rgba.b = color.z();
      rgba.a = color.w();
      marker.points.push_back(origin);
      marker.colors.push_back(rgba);
      marker.points.push_back(tip);
      marker.colors.push_back(rgba);
    };
    add_axis(0, {1.0f, 0.0f, 0.0f, 1.0f});  // X 红
    add_axis(1, {0.0f, 1.0f, 0.0f, 1.0f});  // Y 绿
    add_axis(2, {0.0f, 0.0f, 1.0f, 1.0f});  // Z 蓝

    marker.lifetime = builtin_interfaces::msg::Duration();  // 0 = 永久

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);
    publisher_pool_->Publish(topic, std::make_shared<visualization_msgs::msg::MarkerArray>(markers));
    return marker;
  }

  /// @brief 发布一条规划轨迹供 RViz 播放 (moveit_msgs/DisplayTrajectory)
  /// @param trajectory 待显示的轨迹，首个路点状态会作为 trajectory_start 一并发出，RViz 用它补全缺失的关节
  /// @param topic 发布话题名，需与 RViz 显示插件里的 Trajectory Topic 一致
  void PublishTrajectory(const robot_trajectory::RobotTrajectory &trajectory,
                         const std::string &topic = "planned_path") const {
    moveit_msgs::msg::DisplayTrajectory message;
    moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), message.trajectory_start);
    trajectory.getRobotTrajectoryMsg(message.trajectory.emplace_back());
    trajectory_publisher_pool_->Publish(topic, std::make_shared<moveit_msgs::msg::DisplayTrajectory>(message));
  }

  /// @brief 发布一个机器人状态，RViz 的 moveit_rviz_plugin/RobotState 插件会把它画成一个手臂
  /// @param state 机器人状态（关节角），调用方负责保证已 update()
  /// @param topic 发布话题名，需与 RViz 显示插件里的 Robot State Topic 一致
  /// @param highlight_links 需要高亮上色的 link，用于区分不同阶段的手臂
  /// @param highlight_color 高亮颜色的 RGBA
  void PublishRobotState(const moveit::core::RobotState &state, const std::string &topic = "display_robot_state",
                         const std::vector<std::string> &highlight_links = {},
                         const std::array<float, 4> &highlight_color = {0.2f, 0.8f, 1.0f, 1.0f}) const {
    moveit_msgs::msg::DisplayRobotState message;
    // 抓取规划不涉及附着物体，不用把 attached bodies 一起序列化
    moveit::core::robotStateToRobotStateMsg(state, message.state, false);
    for (const std::string &link : highlight_links) {
      moveit_msgs::msg::ObjectColor highlight;
      highlight.id = link;
      highlight.color.r = highlight_color[0];
      highlight.color.g = highlight_color[1];
      highlight.color.b = highlight_color[2];
      highlight.color.a = highlight_color[3];
      message.highlight_links.push_back(std::move(highlight));
    }
    robot_state_publisher_pool_->Publish(topic, std::make_shared<moveit_msgs::msg::DisplayRobotState>(message));
  }

 private:
  std::shared_ptr<MarkerArrayPublisherPool> publisher_pool_;
  std::shared_ptr<TrajectoryPublisherPool> trajectory_publisher_pool_;
  std::shared_ptr<RobotStatePublisherPool> robot_state_publisher_pool_;
};

}  // namespace amorph_master