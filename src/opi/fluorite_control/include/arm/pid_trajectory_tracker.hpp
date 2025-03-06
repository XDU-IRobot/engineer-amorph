
#ifndef ARM_PID_TRAJECTORY_TRACKER_HPP
#define ARM_PID_TRAJECTORY_TRACKER_HPP

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <limits>
#include <vector>
#include <chrono>

#include <librm.hpp>

#include "common/logger.hpp"

template <int NJoints>
class PidTrajectoryTracker {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  PidTrajectoryTracker(
      std::initializer_list<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>> pids)
      : pids_(pids) {
    if (pids_.size() != 2 * NJoints) {
      error("trajectory tracker: PID number mismatch!");
      throw std::invalid_argument("PID number mismatch!");
    }
    output_torque_.resize(NJoints);
  }
  PidTrajectoryTracker(const PidTrajectoryTracker &) = default;
  PidTrajectoryTracker(PidTrajectoryTracker &&) noexcept = default;

  /**
   * @brief 更新参考轨迹
   * @param trajectory      参考轨迹
   * @param playback_speed  参考轨迹播放速度，范围[0, std::numeric_limits<double>::max()]
   */
  void UpdateTrajectory(const Trajectory &traj, double playback_speed = 1.0) {
    // corner case 1: 空轨迹
    if (traj.points.empty()) {
      warn("trajectory tracker: empty trajectory, rejecting!");
      return;
    }
    // 如果只有一个点，调用另一个重载函数
    if (traj.points.size() == 1) {
      UpdateTrajectory(traj.points[0]);
      return;
    }
    // corner case 2: 时间不单调递增
    for (size_t i = 1; i < traj.points.size(); ++i) {
      if (PointTimeMs(traj.points[i]) <= PointTimeMs(traj.points[i - 1])) {
        warn("trajectory tracker: non-monotonic time in trajectory, rejecting!");
        return;
      }
    }
    // corner case 3: 存在点的关节数量和对象创建时的给定数量不一致
    for (const auto &point : traj.points) {
      if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
        warn("trajectory tracker: joint number mismatch, rejecting!");
        return;
      }
    }

    current_trajectory_ = traj.points;
    playback_speed = std::clamp(playback_speed, 0.0, std::numeric_limits<double>::max());
    const auto time_ratio = 1.0 / playback_speed;
    for (auto &point : current_trajectory_) {
      point.time_from_start.sec *= time_ratio;
      point.time_from_start.nanosec *= time_ratio;
      while (point.time_from_start.nanosec >= 1e9) {
        point.time_from_start.sec += 1;
        point.time_from_start.nanosec -= 1e9;
      }
    }
    ResetReferenceTrajectory();
  }

  /**
   * @brief 更新一条只有一个点的参考轨迹
   * @param point 参考目标点
   */
  void UpdateTrajectory(const Point &point) {
    // corner case: 关节数量和对象创建时的给定数量不一致
    if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
      warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }
    current_trajectory_.clear();
    current_trajectory_.push_back(point);
    ResetReferenceTrajectory();
  }

  /**
   * @brief 重置轨迹（把轨迹的播放进度重置到开始）
   */
  void ResetReferenceTrajectory() {
    ClearPid();
    traj_start_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief 重置所有pid控制器的内部状态
   */
  void ClearPid() {
    for (auto &pid : pids_) {
      pid.Clear();
    }
  }

  /**
   * @brief 更新一步参考点，但不计算PID
   */
  void Update() {
    // corner case 1: 对象刚刚创建，没有轨迹，直接返回
    if (current_trajectory_.empty()) {
      return;
    }
    // corner case 2：只有一个点，直接把index设为0返回
    if (current_trajectory_.size() == 1) {
      current_point_index_ = 0;
      return;
    }

    const auto current_time = std::chrono::steady_clock::now();
    const auto elapsed_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_time - traj_start_time_).count();

    // corner case 3：轨迹播放完毕，直接把index设为最后一个点返回
    if (PointTimeMs(current_trajectory_[current_trajectory_.size() - 1]) < elapsed_time_ms) {
      current_point_index_ = current_trajectory_.size() - 1;
      return;
    }

    // 因为更新轨迹时我们保证时间一定是单调递增的，所以这里可以二分查找最后一个小于等于当前时间的点
    size_t left = 0, right = current_trajectory_.size() - 1;
    while (left < right) {
      const size_t mid = (left + right + 1) / 2;
      if (PointTimeMs(current_trajectory_[mid]) <= elapsed_time_ms) {
        left = mid;
      } else {
        right = mid - 1;
      }
    }
    current_point_index_ = left;
  }

  /**
   * @brief 更新一步
   * @param current_joint_positions     当前关节位置，顺序和轨迹中的关节顺序一致
   * @param current_joint_velocities    当前关节速度，顺序和轨迹中的关节顺序一致
   */
  void Update(std::array<double, NJoints> current_joint_positions,
              std::array<double, NJoints> current_joint_velocities) {
    Update();
    // corner case: 对象刚刚创建，没有轨迹，把输入的当前位置作为目标位置
    if (current_trajectory_.empty()) {
      Point point;
      point.positions.resize(NJoints);
      for (size_t i = 0; i < NJoints; ++i) {
        point.positions[i] = current_joint_positions[i];
      }
      point.velocities.resize(NJoints);
      UpdateTrajectory(point);
      warn("trajectory tracker: no reference trajectory, using current position as reference.");
    }

    // 以当前点为参考计算每个关节的PID
    const auto current_point = current_trajectory_[current_point_index_];
    for (size_t i = 0; i < NJoints; ++i) {
      auto &pid_outer = pids_[2 * i];
      auto &pid_inner = pids_[2 * i + 1];
      pid_outer.Update(current_point.positions[i], current_joint_positions[i]);
      pid_inner.Update(current_point.velocities[i], current_joint_velocities[i]);
      output_torque_[i] = pid_inner.value();
    }
  }

  const auto &output_torque() const { return output_torque_; }
  const auto &trajoctory() const { return current_trajectory_; }
  const auto &current_point() {
    const static auto empty_point = [] {
      Point point;
      point.positions.resize(NJoints);
      point.velocities.resize(NJoints);
      point.accelerations.resize(NJoints);
      point.effort.resize(NJoints);
      return point;
    }();
    Update();
    if (current_trajectory_.empty()) {
      return empty_point;
    } else {
      return current_trajectory_[current_point_index_];
    }
  }

 private:
  int64_t PointTimeMs(const Point &point) const {
    return point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1e6;
  }

 private:
  size_t current_point_index_{};
  std::vector<Point> current_trajectory_{};
  std::chrono::steady_clock::time_point traj_start_time_{};
  std::vector<double> output_torque_{};
  std::vector<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>>
      pids_;  ///< outer0, inner0, outer1, inner1, ...
};

#endif