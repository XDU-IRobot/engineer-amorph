/**
 * @file energy_unit_tracker.cpp
 * @brief EnergyUnitTracker 实现
 */

#include "amorph_rviz_plugins/energy_unit_tracker.hpp"

#include <regex>

namespace amorph {

EnergyUnitTracker::EnergyUnitTracker(rclcpp::Node* node,
                                     std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(node), tf_buffer_(std::move(tf_buffer)) {}

const std::map<std::string, EnergyUnitInfo>& EnergyUnitTracker::scan() {
  units_.clear();

  const auto now = node_->get_clock()->now();
  const auto max_age = rclcpp::Duration::from_seconds(2.0);

  for (int i = 0; i <= 15; ++i) {
    std::string frame_id = kFramePrefix + std::to_string(i) + kFrameSuffix;

    try {
      auto tf = tf_buffer_->lookupTransform("world", frame_id, tf2::TimePointZero,
                                            tf2::durationFromSec(0.010));

      auto stamp = tf.header.stamp;
      auto age = now - stamp;
      if (age > max_age) {
        RCLCPP_DEBUG(node_->get_logger(), "Skipping stale TF %s (age: %.1fs)",
                     frame_id.c_str(), age.seconds());
        continue;
      }

      EnergyUnitInfo info;
      info.frame_id = frame_id;
      info.track_id = i;
      info.pose.position.x = tf.transform.translation.x;
      info.pose.position.y = tf.transform.translation.y;
      info.pose.position.z = tf.transform.translation.z;
      info.pose.orientation = tf.transform.rotation;
      units_[frame_id] = info;
    } catch (const tf2::TransformException&) {
      // Frame not available — skip
    }
  }

  return units_;
}

void EnergyUnitTracker::updatePose(const std::string& frame_id,
                                   const geometry_msgs::msg::Pose& pose) {
  auto it = units_.find(frame_id);
  if (it != units_.end()) {
    it->second.pose = pose;
  }
}

int EnergyUnitTracker::parseTrackId(const std::string& frame_id) {
  if (frame_id.find(kFramePrefix) != 0) return -1;

  auto num_start = std::strlen(kFramePrefix);
  auto num_end = frame_id.find('_', num_start);
  if (num_end == std::string::npos) return -1;

  std::string num_part = frame_id.substr(num_start, num_end - num_start);
  if (num_part.empty() || !std::all_of(num_part.begin(), num_part.end(), ::isdigit)) {
    return -1;
  }
  return std::stoi(num_part);
}

}  // namespace amorph
