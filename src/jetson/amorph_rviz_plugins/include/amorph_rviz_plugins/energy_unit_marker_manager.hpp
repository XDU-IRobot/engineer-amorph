/**
 * @file energy_unit_marker_manager.hpp
 * @brief 能量单元 Interactive Marker 管理器
 *
 * 纯数据→可视化层：接收能量单元列表 + 分配状态，生成/更新 3D 视口中的
 * interactive markers。不关心业务逻辑（谁分配、怎么规划），只负责视觉呈现。
 */

#pragma once

#include <rclcpp/node.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include <map>
#include <string>
#include <memory>
#include <functional>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace amorph {

struct EnergyUnitInfo;

/// marker 被点击时的回调：参数为被点击的能量单元 frame_id
using MarkerClickCallback = std::function<void(const std::string& frame_id)>;

/// marker 被拖拽后位姿更新时的回调
using MarkerPoseUpdateCallback = std::function<void(const std::string& frame_id,
                                                    const geometry_msgs::msg::Pose& new_pose)>;

class EnergyUnitMarkerManager {
 public:
  EnergyUnitMarkerManager(rclcpp::Node* node, const std::string& ns);

  /// 设置点击回调（例如选点分配）
  void setClickCallback(MarkerClickCallback cb) { click_cb_ = std::move(cb); }

  /// 设置拖拽回调（同步位姿）
  void setPoseUpdateCallback(MarkerPoseUpdateCallback cb) { pose_update_cb_ = std::move(cb); }

  /// 根据当前能量单元列表和左右臂分配状态，刷新全部 markers
  void updateMarkers(const std::map<std::string, EnergyUnitInfo>& units,
                     const std::string& left_target,
                     const std::string& right_target,
                     bool left_upward,
                     bool right_upward);

  /// 清空全部 markers
  void clear();

 private:
  void makeMarker(const EnergyUnitInfo& info,
                  bool is_left, bool is_right, bool upward,
                  std::vector<visualization_msgs::msg::InteractiveMarker>& out_markers);

  static visualization_msgs::msg::InteractiveMarkerControl makeArrowControl(bool upward);

  void onFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  rclcpp::Node* node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  MarkerClickCallback click_cb_;
  MarkerPoseUpdateCallback pose_update_cb_;
};

}  // namespace amorph
