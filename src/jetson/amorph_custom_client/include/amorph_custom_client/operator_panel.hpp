/**
 * @file operator_panel.hpp
 * @brief RViz2 Panel 插件 —— 薄协调层
 *
 * 职责：
 * - 连接 UI 信号 → ArmTargetState / EnergyUnitTracker / EnergyUnitMarkerManager / MtcRunner
 * - 持有 ROS 资源（tf_buffer/listener/broadcaster），分发给子模块
 * - 不包含 TF 扫描细节、marker 创建细节、MTC 流水线细节
 */

#pragma once

#ifndef Q_MOC_RUN
#  include <rclcpp/node.hpp>
#  include <tf2_ros/buffer.h>
#  include <tf2_ros/transform_listener.h>
#  include <tf2_ros/transform_broadcaster.h>
#endif

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

#include <string>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>

#include "amorph_custom_client/operator_panel_ui.hpp"
#include "amorph_custom_client/arm_target_state.hpp"
#include "amorph_custom_client/energy_unit_tracker.hpp"
#include "amorph_custom_client/energy_unit_marker_manager.hpp"
#include "amorph_custom_client/mtc_runner.hpp"

namespace amorph {

class OperatorPanelUi;
class EnergyUnitTracker;
class EnergyUnitMarkerManager;
class ArmTargetState;

class OperatorPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit OperatorPanel(QWidget* parent = nullptr);
  ~OperatorPanel() override;

  void onInitialize() override;

 private Q_SLOTS:
  void onRefresh();

  void onLeftSetToggled(bool checked);
  void onRightSetToggled(bool checked);

  void onLeftUp();
  void onLeftDown();
  void onRightUp();
  void onRightDown();

  void onPlanLeft();
  void onPlanRight();

 private:
  /// 扫描 TF → 更新 markers → 更新 UI
  void doScanAndUpdate();

  /// 同步 markers（传入当前分配状态）
  void syncMarkers();

  /// marker 被点击 → 尝试分配给当前选点模式对应的臂
  void onMarkerClicked(const std::string& frame_id);

  /// marker 被拖拽 → 同步位姿
  void onMarkerPoseUpdated(const std::string& frame_id,
                           const geometry_msgs::msg::Pose& pose);

  /// 进入选点模式
  void enterSelectionMode(SelectionMode mode);

  /// 退出选点模式
  void exitSelectionMode();

  /// 更新 UI 文字和按钮状态
  void updateUIState();

  /// 广播目标 TF（供 MTC 使用）
  void broadcastTargetTF(const std::string& child_frame,
                         const geometry_msgs::msg::Pose& pose);

  /// 启动 MTC 规划（创建异步任务或直接同步运行）
  void runPlan(bool left);

  // ── 子模块 ──
  std::unique_ptr<OperatorPanelUi> ui_;
  std::unique_ptr<EnergyUnitTracker> tracker_;
  std::unique_ptr<EnergyUnitMarkerManager> marker_mgr_;

  // ── 双臂目标状态 ──
  std::unique_ptr<ArmTargetState> left_arm_;
  std::unique_ptr<ArmTargetState> right_arm_;

  // ── ROS 资源 ──
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ── 选点模式 ──
  SelectionMode selection_mode_ = SelectionMode::None;
};

}  // namespace amorph
