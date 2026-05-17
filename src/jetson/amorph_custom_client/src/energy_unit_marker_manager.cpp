/**
 * @file energy_unit_marker_manager.cpp
 * @brief EnergyUnitMarkerManager 实现
 */

#include "amorph_custom_client/energy_unit_marker_manager.hpp"
#include "amorph_custom_client/energy_unit_tracker.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace amorph {

namespace vm = visualization_msgs::msg;

EnergyUnitMarkerManager::EnergyUnitMarkerManager(rclcpp::Node* node, const std::string& ns)
    : node_(node),
      im_server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(ns, node)) {}

void EnergyUnitMarkerManager::clear() {
  im_server_->clear();
  im_server_->applyChanges();
}

void EnergyUnitMarkerManager::updateMarkers(
    const std::map<std::string, EnergyUnitInfo>& units,
    const std::string& left_target,
    const std::string& right_target,
    bool left_upward,
    bool right_upward) {

  im_server_->clear();

  std::vector<vm::InteractiveMarker> markers;
  for (const auto& [frame_id, info] : units) {
    bool is_left = (frame_id == left_target);
    bool is_right = (frame_id == right_target);
    bool upward = is_left ? left_upward : (is_right ? right_upward : false);
    makeMarker(info, is_left, is_right, upward, markers);
  }

  // 批量插入所有 markers（带回调），然后统一 applyChanges
  for (auto& m : markers) {
    if (click_cb_ || pose_update_cb_) {
      im_server_->insert(m,
          [this](const vm::InteractiveMarkerFeedback::ConstSharedPtr& fb) { onFeedback(fb); },
          vm::InteractiveMarkerFeedback::POSE_UPDATE |
          vm::InteractiveMarkerFeedback::MOUSE_UP);
    } else {
      im_server_->insert(m);
    }
  }
  im_server_->applyChanges();
}

void EnergyUnitMarkerManager::makeMarker(
    const EnergyUnitInfo& info,
    bool is_left, bool is_right, bool upward,
    std::vector<vm::InteractiveMarker>& out_markers) {

  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.name = info.frame_id;
  int_marker.description = "能量单元 #" + std::to_string(info.track_id);
  int_marker.pose = info.pose;
  int_marker.scale = 0.20;

  // ── 逐轴平移控件 ──
  for (const auto& [ox, oy, oz] : {
         std::tuple{1.0, 0.0, 0.0},
         std::tuple{0.0, 1.0, 0.0},
         std::tuple{0.0, 0.0, 1.0}}) {
    vm::InteractiveMarkerControl ctrl;
    ctrl.name = "move_" + std::to_string(static_cast<int>(ox));
    ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    ctrl.orientation.x = ox;
    ctrl.orientation.y = oy;
    ctrl.orientation.z = oz;
    ctrl.orientation.w = 1.0;
    int_marker.controls.push_back(ctrl);
  }

  // ── STL 模型 ──
  vm::Marker mesh;
  mesh.type = vm::Marker::MESH_RESOURCE;
  try {
    auto share_dir = ament_index_cpp::get_package_share_directory("amorph_assets");
    mesh.mesh_resource = "file://" + share_dir + "/assets/energy_unit.stl";
    mesh.mesh_use_embedded_materials = false;
  } catch (...) {
    mesh.type = vm::Marker::CUBE;
  }
  mesh.scale.x = 1.0;
  mesh.scale.y = 1.0;
  mesh.scale.z = 1.0;

  if (is_left) {
    mesh.color.r = 0.0f; mesh.color.g = 0.5f; mesh.color.b = 1.0f;
  } else if (is_right) {
    mesh.color.r = 1.0f; mesh.color.g = 0.5f; mesh.color.b = 0.0f;
  } else {
    mesh.color.r = 0.0f; mesh.color.g = 1.0f; mesh.color.b = 0.0f;
  }
  mesh.color.a = 0.65f;

  vm::InteractiveMarkerControl mesh_ctrl;
  mesh_ctrl.name = "select_" + info.frame_id;
  mesh_ctrl.always_visible = true;
  mesh_ctrl.interaction_mode = vm::InteractiveMarkerControl::BUTTON;
  mesh_ctrl.markers.push_back(mesh);
  int_marker.controls.push_back(mesh_ctrl);

  // ── 已分配 → 添加方向箭头 ──
  if (is_left || is_right) {
    int_marker.controls.push_back(makeArrowControl(upward));
  }

  out_markers.push_back(std::move(int_marker));
}

vm::InteractiveMarkerControl EnergyUnitMarkerManager::makeArrowControl(bool upward) {
  vm::InteractiveMarkerControl arrow_ctrl;
  arrow_ctrl.name = "lift_arrow";
  arrow_ctrl.interaction_mode = vm::InteractiveMarkerControl::BUTTON;

  vm::Marker arrow;
  arrow.type = vm::Marker::ARROW;
  arrow.scale.x = 0.5f;
  arrow.scale.y = 0.04f;
  arrow.scale.z = 0.04f;
  arrow.color.r = 1.0f;
  arrow.color.g = 1.0f;
  arrow.color.b = 0.0f;
  arrow.color.a = 0.9f;

  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = upward ? -s : s;  // +z (up) / -z (down)
  arrow.pose.orientation.z = 0.0;
  arrow.pose.orientation.w = c;
  arrow.pose.position.z = 0.15;

  arrow_ctrl.markers.push_back(arrow);
  return arrow_ctrl;
}

void EnergyUnitMarkerManager::onFeedback(
    const vm::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  if (!feedback) return;

  const auto& frame_id = feedback->marker_name;

  if (feedback->event_type == vm::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (pose_update_cb_) {
      pose_update_cb_(frame_id, feedback->pose);
    }
    return;
  }

  // MOUSE_UP: 点击 → 选点
  if (feedback->event_type == vm::InteractiveMarkerFeedback::MOUSE_UP) {
    if (click_cb_) {
      click_cb_(frame_id);
    }
  }
}

}  // namespace amorph
