/**
 * @file operator_panel.cpp
 * @brief 能量单元操作面板 —— 薄协调层
 *
 * ┌──────────────┐    ┌───────────────────┐    ┌──────────────────────────┐
 * │ OperatorPanel│───▶│ EnergyUnitTracker │───▶│ tf2_ros::Buffer          │
 * │   (协调层)    │    │  (TF 扫描+缓存)    │    └──────────────────────────┘
 * │              │    └───────────────────┘
 * │              │    ┌──────────────────────┐
 * │              │───▶│ EnergyUnitMarkerMgr  │───▶ InteractiveMarkerServer
 * │              │    │  (可视化)             │    (3D 视口 markers)
 * │              │    └──────────────────────┘
 * │              │    ┌──────────────────────┐
 * │              │───▶│ ArmTargetState ×2     │   纯数据: target_id + lift_upward
 * │              │    └──────────────────────┘
 * │              │    ┌──────────────────────┐
 * │              │───▶│ MtcRunner            │───▶ PickEnergyUnitTask
 * │              │    └──────────────────────┘    (MoveIt Task Constructor)
 * └──────────────┘
 */

#include "amorph_custom_client/operator_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(amorph::OperatorPanel, rviz_common::Panel)

namespace amorph {

// ═══════════════════════════════════════════════════════════════
// 构造 / 析构 / onInitialize
// ═══════════════════════════════════════════════════════════════

OperatorPanel::OperatorPanel(QWidget* parent) : rviz_common::Panel(parent) {}

OperatorPanel::~OperatorPanel() = default;

void OperatorPanel::onInitialize() {
  auto ros_abstraction = getDisplayContext()->getRosNodeAbstraction();
  if (auto locked = ros_abstraction.lock()) {
    node_ = locked->get_raw_node();
  }
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("amorph_operator_panel"),
                 "Failed to get ROS node from RViz");
    return;
  }

  // ── TF 基础设施（tracker 和 marker_mgr 共享 buffer） ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // ── 双臂目标状态（纯数据，左右对称） ──
  left_arm_ = std::make_unique<ArmTargetState>();
  right_arm_ = std::make_unique<ArmTargetState>();

  // 分配完成时自动刷新 markers + UI
  auto on_assigned = [this](const std::string&) {
    syncMarkers();
    updateUIState();
  };
  left_arm_->setAssignCallback(on_assigned);
  right_arm_->setAssignCallback(on_assigned);

  // ── 能量单元跟踪器（TF 扫描） ──
  tracker_ = std::make_unique<EnergyUnitTracker>(node_.get(), tf_buffer_);

  // ── Interactive Marker 管理器（3D 可视化） ──
  marker_mgr_ = std::make_unique<EnergyUnitMarkerManager>(node_.get(), "amorph_operator");
  marker_mgr_->setClickCallback(
      [this](const std::string& frame_id) { onMarkerClicked(frame_id); });
  marker_mgr_->setPoseUpdateCallback(
      [this](const std::string& frame_id, const geometry_msgs::msg::Pose& pose) {
        onMarkerPoseUpdated(frame_id, pose);
      });

  // ── UI ──
  ui_ = std::make_unique<OperatorPanelUi>();
  ui_->setupUi(this);

  // ── 信号连接 ──
  connect(ui_->refreshBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onRefresh);

  connect(ui_->leftSetBtn(), &QPushButton::toggled,
          this, &OperatorPanel::onLeftSetToggled);
  connect(ui_->leftUpBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onLeftUp);
  connect(ui_->leftDownBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onLeftDown);
  connect(ui_->planLeftBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onPlanLeft);

  connect(ui_->rightSetBtn(), &QPushButton::toggled,
          this, &OperatorPanel::onRightSetToggled);
  connect(ui_->rightUpBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onRightUp);
  connect(ui_->rightDownBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onRightDown);
  connect(ui_->planRightBtn(), &QPushButton::clicked,
          this, &OperatorPanel::onPlanRight);

  RCLCPP_INFO(node_->get_logger(), "OperatorPanel initialized.");
}

// ═══════════════════════════════════════════════════════════════
// 刷新：扫描 TF → 更新 markers → 更新 UI
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::doScanAndUpdate() {
  if (!tracker_ || !node_) return;

  tracker_->scan();

  const auto& units = tracker_->units();
  if (units.empty()) {
    ui_->detectionLabel()->setText("未检测到能量单元");
    ui_->statusLabel()->setText("⚠ 未发现 energy_unit_*_smoothed TF 帧");
    updateUIState();
    return;
  }

  ui_->detectionLabel()->setText(
      QString("检测到 %1 个能量单元").arg(static_cast<int>(units.size())));

  syncMarkers();
  updateUIState();

  ui_->statusLabel()->setText(
      QString("已刷新 · 共 %1 个能量单元，等待分配目标")
          .arg(static_cast<int>(units.size())));

  RCLCPP_INFO(node_->get_logger(), "Refreshed: found %zu energy units", units.size());
}

void OperatorPanel::onRefresh() { doScanAndUpdate(); }

void OperatorPanel::syncMarkers() {
  if (!marker_mgr_) return;
  marker_mgr_->updateMarkers(
      tracker_->units(),
      left_arm_->targetFrameId(), right_arm_->targetFrameId(),
      left_arm_->liftUpward(), right_arm_->liftUpward());
}

// ═══════════════════════════════════════════════════════════════
// marker 回调
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::onMarkerClicked(const std::string& frame_id) {
  if (selection_mode_ == SelectionMode::None) {
    RCLCPP_WARN(node_->get_logger(),
                "Marker click ignored: not in selection mode");
    return;
  }

  auto it = tracker_->find(frame_id);
  if (it == tracker_->units().end()) {
    RCLCPP_WARN(node_->get_logger(),
                "Marker click ignored: frame %s not tracked", frame_id.c_str());
    return;
  }

  const bool left = (selection_mode_ == SelectionMode::Left);
  auto& arm = left ? *left_arm_ : *right_arm_;
  arm.assignTarget(frame_id);

  // 广播目标 TF
  broadcastTargetTF("energy_unit_selected", it->second.pose);
  broadcastTargetTF(left ? "energy_unit_left_target" : "energy_unit_right_target",
                    it->second.pose);

  ui_->statusLabel()->setText(
      QString("%1目标已分配: 能量单元 #%2")
          .arg(left ? "左臂" : "右臂")
          .arg(it->second.track_id));

  exitSelectionMode();
}

void OperatorPanel::onMarkerPoseUpdated(const std::string& frame_id,
                                        const geometry_msgs::msg::Pose& pose) {
  tracker_->updatePose(frame_id, pose);
  // 如果被拖拽的恰好是某臂的目标，同步广播 TF
  if (frame_id == left_arm_->targetFrameId() || frame_id == right_arm_->targetFrameId()) {
    broadcastTargetTF("energy_unit_selected", pose);
  }
}

// ═══════════════════════════════════════════════════════════════
// 选点模式
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::enterSelectionMode(SelectionMode mode) {
  selection_mode_ = mode;
  const char* arm = (mode == SelectionMode::Left) ? "左臂" : "右臂";
  const char* color = (mode == SelectionMode::Left) ? "🔵" : "🟠";
  ui_->statusLabel()->setText(
      QString("%1 %2选点模式: 请点击 3D 视口中的能量单元").arg(color, arm));
}

void OperatorPanel::exitSelectionMode() {
  selection_mode_ = SelectionMode::None;
  ui_->leftSetBtn()->setChecked(false);
  ui_->rightSetBtn()->setChecked(false);
  updateUIState();
}

// ═══════════════════════════════════════════════════════════════
// UI 回调 —— Set / ▲ / ▼
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::onLeftSetToggled(bool checked) {
  if (checked) {
    ui_->rightSetBtn()->setChecked(false);
    enterSelectionMode(SelectionMode::Left);
  } else if (selection_mode_ == SelectionMode::Left) {
    exitSelectionMode();
  }
  updateUIState();
}

void OperatorPanel::onRightSetToggled(bool checked) {
  if (checked) {
    ui_->leftSetBtn()->setChecked(false);
    enterSelectionMode(SelectionMode::Right);
  } else if (selection_mode_ == SelectionMode::Right) {
    exitSelectionMode();
  }
  updateUIState();
}

void OperatorPanel::onLeftUp() {
  left_arm_->setLiftUpward(true);
  ui_->leftUpBtn()->setChecked(true);
  ui_->leftDownBtn()->setChecked(false);
  syncMarkers();
  updateUIState();
}

void OperatorPanel::onLeftDown() {
  left_arm_->setLiftUpward(false);
  ui_->leftDownBtn()->setChecked(true);
  ui_->leftUpBtn()->setChecked(false);
  syncMarkers();
  updateUIState();
}

void OperatorPanel::onRightUp() {
  right_arm_->setLiftUpward(true);
  ui_->rightUpBtn()->setChecked(true);
  ui_->rightDownBtn()->setChecked(false);
  syncMarkers();
  updateUIState();
}

void OperatorPanel::onRightDown() {
  right_arm_->setLiftUpward(false);
  ui_->rightDownBtn()->setChecked(true);
  ui_->rightUpBtn()->setChecked(false);
  syncMarkers();
  updateUIState();
}

// ═══════════════════════════════════════════════════════════════
// MTC 规划
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::onPlanLeft() { runPlan(true); }
void OperatorPanel::onPlanRight() { runPlan(false); }

void OperatorPanel::runPlan(bool left) {
  auto& arm = left ? *left_arm_ : *right_arm_;
  const char* arm_name = left ? "左臂" : "右臂";

  if (!arm.hasTarget()) {
    ui_->statusLabel()->setText(
        QString("⚠ %1 未分配目标能量单元").arg(arm_name));
    return;
  }

  const auto& frame_id = arm.targetFrameId();
  auto it = tracker_->find(frame_id);
  if (it == tracker_->units().end()) {
    arm.clearTarget();
    ui_->statusLabel()->setText(
        QString("⚠ %1 目标能量单元已丢失").arg(arm_name));
    updateUIState();
    return;
  }

  ui_->statusLabel()->setText(
      QString("⏳ %1 MTC 规划中...").arg(arm_name));

  auto runner = std::make_shared<MtcRunner>(
      node_.get(), left ? "pick_eu_left" : "pick_eu_right");
  runner->setObjectPose(it->second.pose);
  runner->setLiftDirection(arm.liftUpward());
  runner->setPlanCallback([this, arm_name, track_id = it->second.track_id]
                          (bool success, const std::string& message) {
    if (success) {
      ui_->statusLabel()->setText(
          QString("✅ %1 规划成功 · 能量单元 #%2").arg(arm_name).arg(track_id));
    } else {
      ui_->statusLabel()->setText(
          QString("❌ %1 %2").arg(arm_name).arg(message.c_str()));
    }
  });

  runner->run();
}

// ═══════════════════════════════════════════════════════════════
// UI 状态
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::updateUIState() {
  bool has_units = !tracker_->units().empty();

  auto update_arm_label = [&](bool left) {
    auto& arm = left ? *left_arm_ : *right_arm_;
    auto* label = left ? ui_->leftTargetLabel() : ui_->rightTargetLabel();
    const char* color = left ? "#2980b9" : "#d35400";
    bool upward = arm.liftUpward();

    if (arm.hasTarget()) {
      const auto& target = arm.targetFrameId();
      auto it = tracker_->find(target);
      if (it != tracker_->units().end()) {
        label->setText(QString("目标: #%1 · 方向: %2")
                           .arg(it->second.track_id)
                           .arg(upward ? "▲ 抓取" : "▼ 放置"));
        label->setStyleSheet(
            QString("QLabel { color: %1; font-weight: bold; padding: 2px; }").arg(color));
      } else {
        label->setText("目标: (已丢失)");
      }
    } else {
      label->setText(QString("目标: 未分配 · 方向: %1")
                         .arg(upward ? "▲ 抓取" : "▼ 放置"));
      label->setStyleSheet("QLabel { color: #7f8c8d; font-style: italic; padding: 2px; }");
    }
  };

  update_arm_label(true);
  update_arm_label(false);

  ui_->leftSetBtn()->setEnabled(has_units);
  ui_->rightSetBtn()->setEnabled(has_units);
  ui_->leftUpBtn()->setEnabled(has_units);
  ui_->leftDownBtn()->setEnabled(has_units);
  ui_->rightUpBtn()->setEnabled(has_units);
  ui_->rightDownBtn()->setEnabled(has_units);

  ui_->planLeftBtn()->setEnabled(has_units && left_arm_->hasTarget());
  ui_->planRightBtn()->setEnabled(has_units && right_arm_->hasTarget());
}

// ═══════════════════════════════════════════════════════════════
// TF 广播
// ═══════════════════════════════════════════════════════════════

void OperatorPanel::broadcastTargetTF(const std::string& child_frame,
                                      const geometry_msgs::msg::Pose& pose) {
  if (!tf_broadcaster_) return;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = child_frame;
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

}  // namespace amorph
