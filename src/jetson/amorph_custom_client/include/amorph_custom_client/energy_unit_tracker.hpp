/**
 * @file energy_unit_tracker.hpp
 * @brief 能量单元跟踪器 —— 从 TF 发现 energy_unit_*_smoothed 帧
 *
 * 纯数据层，不依赖 Qt，不持有 ROS 资源所有权（接收共享指针）。
 */

#pragma once

#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>

#include <map>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace amorph {

/// 检测到的能量单元信息
struct EnergyUnitInfo {
  std::string frame_id;           // e.g. "energy_unit_1_smoothed"
  int track_id = -1;              // 解析出的 track_id
  geometry_msgs::msg::Pose pose;  // 当前位姿（world 系）
};

class EnergyUnitTracker {
 public:
  EnergyUnitTracker(rclcpp::Node* node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /// 扫描 TF，刷新内部缓存，返回当前检测结果
  const std::map<std::string, EnergyUnitInfo>& scan();

  /// 获取当前缓存的全部能量单元
  const std::map<std::string, EnergyUnitInfo>& units() const { return units_; }

  /// 查找指定帧
  auto find(const std::string& frame_id) const { return units_.find(frame_id); }

  /// 更新某个单元的位姿（例如 interactive marker 拖拽后回写）
  void updatePose(const std::string& frame_id, const geometry_msgs::msg::Pose& pose);

  /// 从 TF 帧名解析 track_id（"energy_unit_3_smoothed" → 3）
  static int parseTrackId(const std::string& frame_id);

  /// 帧名 → track_id 前缀
  static constexpr const char* kFramePrefix = "energy_unit_";
  static constexpr const char* kFrameSuffix = "_smoothed";

 private:
  rclcpp::Node* node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::map<std::string, EnergyUnitInfo> units_;
};

}  // namespace amorph
