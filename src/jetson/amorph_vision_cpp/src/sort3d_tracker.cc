#include "amorph_vision_cpp/sort3d_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "amorph_vision_cpp/hungarian.hpp"

namespace amorph::vision {
namespace {

Eigen::Matrix<double, 6, 6> BuildTransition(double dt) {
  Eigen::Matrix<double, 6, 6> f = Eigen::Matrix<double, 6, 6>::Identity();
  f(0, 3) = dt;
  f(1, 4) = dt;
  f(2, 5) = dt;
  return f;
}

/// 常速度模型下白噪声加速度离散化的过程噪声协方差。
Eigen::Matrix<double, 6, 6> BuildProcessNoise(double dt, double process_var) {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  Eigen::Matrix2d q_block;
  q_block << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2;
  q_block *= process_var;

  Eigen::Matrix<double, 6, 6> q = Eigen::Matrix<double, 6, 6>::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    q(axis, axis) = q_block(0, 0);
    q(axis, axis + 3) = q_block(0, 1);
    q(axis + 3, axis) = q_block(1, 0);
    q(axis + 3, axis + 3) = q_block(1, 1);
  }
  return q;
}

/// 观测矩阵：只观测位置。
Eigen::Matrix<double, 3, 6> BuildObservation() {
  Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  h(2, 2) = 1.0;
  return h;
}

}  // namespace

void Sort3DTracker::KalmanTrack3D::Predict(double dt, double process_var) {
  const Eigen::Matrix<double, 6, 6> f = BuildTransition(dt);
  state = f * state;
  covariance = f * covariance * f.transpose() + BuildProcessNoise(dt, process_var);
  ++age;
  ++missed;
}

void Sort3DTracker::KalmanTrack3D::Update(const Eigen::Vector3d& measurement, const BoundingBox& bbox,
                                          double measurement_var) {
  const Eigen::Matrix<double, 3, 6> h = BuildObservation();
  const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * measurement_var;

  const Eigen::Vector3d innovation = measurement - h * state;
  const Eigen::Matrix3d s = h * covariance * h.transpose() + r;
  const Eigen::Matrix<double, 6, 3> k = covariance * h.transpose() * s.inverse();

  state += k * innovation;
  covariance = (Eigen::Matrix<double, 6, 6>::Identity() - k * h) * covariance;

  last_bbox = bbox;
  ++hits;
  missed = 0;
}

double Sort3DTracker::KalmanTrack3D::MahalanobisSquared(const Eigen::Vector3d& measurement,
                                                        double measurement_var) const {
  const Eigen::Matrix<double, 3, 6> h = BuildObservation();
  const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * measurement_var;

  const Eigen::Vector3d innovation = measurement - h * state;
  const Eigen::Matrix3d s = h * covariance * h.transpose() + r;
  return innovation.transpose() * s.inverse() * innovation;
}

void Sort3DTracker::Reset() {
  tracks_.clear();
  available_ids_.clear();
  for (int id = 0; id < kMaxTracks; ++id) {
    available_ids_.push_back(id);
  }
  lru_order_.clear();
}

std::vector<TrackAssignment> Sort3DTracker::Update(const std::vector<Detection>& detections, double dt,
                                                   std::vector<int>* removed_ids) {
  removed_ids->clear();
  dt = std::max(1e-3, dt);

  for (auto& [id, track] : tracks_) {
    track.Predict(dt, process_var_);
  }

  if (detections.empty()) {
    *removed_ids = PruneDeadTracks();
    return {};
  }

  std::vector<Eigen::Vector3d> positions;
  std::vector<BoundingBox> boxes;
  positions.reserve(detections.size());
  boxes.reserve(detections.size());
  for (const Detection& detection : detections) {
    positions.push_back(detection.position);
    boxes.push_back(detection.bbox);
  }

  std::vector<TrackAssignment> assignments;

  if (tracks_.empty()) {
    for (std::size_t i = 0; i < detections.size(); ++i) {
      const CreateResult result = CreateTrack(positions[i], boxes[i]);
      if (result.track_id < 0) {
        break;  // id 池耗尽
      }
      if (min_hits_ <= 1) {
        TrackAssignment assignment;
        assignment.track_id = result.track_id;
        assignment.detection_index = static_cast<int>(i);
        assignment.filtered_position = positions[i];  // 新轨迹直接以测量值初始化
        assignments.push_back(assignment);
      }
    }
    return assignments;
  }

  std::vector<int> track_ids;
  track_ids.reserve(tracks_.size());
  for (const auto& [id, track] : tracks_) {
    track_ids.push_back(id);
  }

  const std::vector<std::vector<double>> cost = BuildCostMatrix(track_ids, positions, boxes);

  // gating 可能让整行/整列都为 inf，先取可行子矩阵再解
  const std::size_t rows = cost.size();
  const std::size_t cols = cost.front().size();
  std::vector<std::size_t> feasible_rows;
  std::vector<std::size_t> feasible_cols;
  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      if (std::isfinite(cost[i][j])) {
        if (std::find(feasible_rows.begin(), feasible_rows.end(), i) == feasible_rows.end()) {
          feasible_rows.push_back(i);
        }
        if (std::find(feasible_cols.begin(), feasible_cols.end(), j) == feasible_cols.end()) {
          feasible_cols.push_back(j);
        }
      }
    }
  }

  std::vector<std::pair<std::size_t, std::size_t>> matches;
  if (!feasible_rows.empty() && !feasible_cols.empty()) {
    std::vector<std::vector<double>> reduced(feasible_rows.size(), std::vector<double>(feasible_cols.size(), 0.0));
    for (std::size_t i = 0; i < feasible_rows.size(); ++i) {
      for (std::size_t j = 0; j < feasible_cols.size(); ++j) {
        reduced[i][j] = cost[feasible_rows[i]][feasible_cols[j]];
      }
    }
    const std::vector<int> solution = SolveAssignment(reduced);
    for (std::size_t i = 0; i < solution.size(); ++i) {
      if (solution[i] < 0) {
        continue;
      }
      matches.emplace_back(feasible_rows[i], feasible_cols[static_cast<std::size_t>(solution[i])]);
    }
  }

  std::vector<bool> matched_track(track_ids.size(), false);
  std::vector<bool> matched_detection(detections.size(), false);
  std::vector<int> evicted_ids;

  for (const auto& [row, col] : matches) {
    if (!std::isfinite(cost[row][col])) {
      continue;
    }
    const int track_id = track_ids[row];
    auto track_it = tracks_.find(track_id);
    if (track_it == tracks_.end()) {
      continue;
    }

    // gating 已保证在门限内，这里再校验一次做防御
    if (track_it->second.MahalanobisSquared(positions[col], measurement_var_) > mahalanobis_thresh_) {
      continue;
    }

    track_it->second.Update(positions[col], boxes[col], measurement_var_);
    MarkMostRecentlyUsed(track_id);
    matched_track[row] = true;
    matched_detection[col] = true;

    if (track_it->second.hits >= min_hits_) {
      TrackAssignment assignment;
      assignment.track_id = track_id;
      assignment.detection_index = static_cast<int>(col);
      // KF 更新后的后验位置，比单帧测量平滑
      assignment.filtered_position = track_it->second.state.head<3>();
      assignments.push_back(assignment);
    }
  }

  std::vector<int> unmatched_track_ids;
  for (std::size_t i = 0; i < track_ids.size(); ++i) {
    if (!matched_track[i]) {
      unmatched_track_ids.push_back(track_ids[i]);
    }
  }

  // 未匹配的检测新建轨迹
  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (matched_detection[i]) {
      continue;
    }
    const CreateResult result = CreateTrack(positions[i], boxes[i]);
    if (result.track_id < 0) {
      continue;  // id 池满且所有轨迹均活跃，丢弃该检测
    }
    if (result.evicted_id.has_value()) {
      evicted_ids.push_back(*result.evicted_id);
    }
    if (min_hits_ <= 1) {
      TrackAssignment assignment;
      assignment.track_id = result.track_id;
      assignment.detection_index = static_cast<int>(i);
      assignment.filtered_position = positions[i];  // 新轨迹直接以测量值初始化
      assignments.push_back(assignment);
    }
  }

  *removed_ids = PruneSpecificTracks(unmatched_track_ids);
  removed_ids->insert(removed_ids->end(), evicted_ids.begin(), evicted_ids.end());

  return assignments;
}

Sort3DTracker::CreateResult Sort3DTracker::CreateTrack(const Eigen::Vector3d& position, const BoundingBox& bbox) {
  CreateResult result;

  int track_id = -1;
  if (!available_ids_.empty()) {
    track_id = available_ids_.front();
    available_ids_.erase(available_ids_.begin());
  } else {
    // 淘汰最久未使用且本帧未命中的轨迹
    auto evict_it = lru_order_.end();
    for (auto it = lru_order_.begin(); it != lru_order_.end(); ++it) {
      const auto track_it = tracks_.find(*it);
      if (track_it != tracks_.end() && track_it->second.missed > 0) {
        evict_it = it;
        break;
      }
    }
    if (evict_it == lru_order_.end()) {
      return result;  // 无可用 id
    }
    track_id = *evict_it;
    lru_order_.erase(evict_it);
    tracks_.erase(track_id);
    result.evicted_id = track_id;
  }

  KalmanTrack3D track;
  track.track_id = track_id;
  track.state.head<3>() = position;
  track.covariance = Eigen::Matrix<double, 6, 6>::Zero();
  track.covariance.diagonal() << 0.05, 0.05, 0.05, 1.0, 1.0, 1.0;
  track.last_bbox = bbox;
  tracks_[track_id] = track;
  MarkMostRecentlyUsed(track_id);

  result.track_id = track_id;
  return result;
}

void Sort3DTracker::MarkMostRecentlyUsed(int track_id) {
  const auto it = std::find(lru_order_.begin(), lru_order_.end(), track_id);
  if (it != lru_order_.end()) {
    lru_order_.erase(it);
  }
  lru_order_.push_back(track_id);
}

std::vector<int> Sort3DTracker::PruneDeadTracks() {
  std::vector<int> removed;
  std::vector<int> track_ids;
  track_ids.reserve(tracks_.size());
  for (const auto& [id, track] : tracks_) {
    track_ids.push_back(id);
  }
  for (const int track_id : track_ids) {
    const auto it = tracks_.find(track_id);
    if (it != tracks_.end() && it->second.missed > max_age_) {
      removed.push_back(track_id);
      ReleaseTrackId(track_id);
    }
  }
  return removed;
}

std::vector<int> Sort3DTracker::PruneSpecificTracks(const std::vector<int>& track_ids) {
  std::vector<int> removed;
  for (const int track_id : track_ids) {
    const auto it = tracks_.find(track_id);
    if (it == tracks_.end()) {
      continue;
    }
    if (it->second.missed > max_age_) {
      removed.push_back(track_id);
      ReleaseTrackId(track_id);
    }
  }
  return removed;
}

void Sort3DTracker::ReleaseTrackId(int track_id) {
  tracks_.erase(track_id);
  const auto it = std::find(lru_order_.begin(), lru_order_.end(), track_id);
  if (it != lru_order_.end()) {
    lru_order_.erase(it);
  }
  if (std::find(available_ids_.begin(), available_ids_.end(), track_id) == available_ids_.end()) {
    available_ids_.push_back(track_id);
  }
}

std::vector<std::vector<double>> Sort3DTracker::BuildCostMatrix(const std::vector<int>& track_ids,
                                                                const std::vector<Eigen::Vector3d>& positions,
                                                                const std::vector<BoundingBox>& boxes) const {
  const double pos_weight = std::clamp(1.0 - iou_weight_, 0.0, 1.0);
  const double iou_weight = std::clamp(iou_weight_, 0.0, 1.0);

  std::vector<std::vector<double>> cost(track_ids.size(),
                                        std::vector<double>(positions.size(), std::numeric_limits<double>::infinity()));
  for (std::size_t i = 0; i < track_ids.size(); ++i) {
    const auto it = tracks_.find(track_ids[i]);
    if (it == tracks_.end()) {
      continue;
    }
    const KalmanTrack3D& track = it->second;
    for (std::size_t j = 0; j < positions.size(); ++j) {
      const double d2 = track.MahalanobisSquared(positions[j], measurement_var_);
      if (d2 > mahalanobis_thresh_) {
        continue;
      }
      const double pos_cost = d2 / mahalanobis_thresh_;
      const double iou = track.last_bbox.Iou(boxes[j]);
      cost[i][j] = pos_weight * pos_cost + iou_weight * (1.0 - iou);
    }
  }
  return cost;
}

}  // namespace amorph::vision
