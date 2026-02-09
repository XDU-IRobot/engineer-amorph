#pragma once

#include <atomic>
#include <vector>
#include <chrono>
#include <utility>
#include <initializer_list>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "joint_space_interpolator.hpp"
#include "logger.hpp"

namespace amorph {

/**
 * @brief   用来沿时间进度“播放”一条Multi-dof轨迹的Tracker，命名灵感来自那种老电脑上做音乐的Tracker :))
 * @tparam  NJoints 关节数量
 */
template <int NJoints>
class TrajectoryTracker {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;  ///< 轨迹
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;  ///< 轨迹点

 public:
  TrajectoryTracker(std::initializer_list<std::pair<float, float>> joint_limits) : joint_limits_(joint_limits) {
    if (joint_limits_.size() != NJoints) {
      L::F("trajectory tracker: number of pairs of joint limit mismatch! expected {}, got {}", NJoints,
           joint_limits_.size());
      throw std::invalid_argument("Joint limits size mismatch");
    }
    // 初始化 current_point_ 到安全状态
    current_point_.positions.resize(NJoints);
    current_point_.velocities.resize(NJoints);
    current_point_.accelerations.resize(NJoints);
    current_point_.effort.resize(NJoints);
    // 将初始位置设置为各关节限位的中间值
    for (size_t i = 0; i < NJoints; ++i) {
      current_point_.positions[i] = (joint_limits_[i].first + joint_limits_[i].second) / 2.0f;
      current_point_.velocities[i] = 0.0f;
      current_point_.accelerations[i] = 0.0f;
      current_point_.effort[i] = 0.0f;
    }
  }
  TrajectoryTracker(const TrajectoryTracker &) = delete;  // 禁止拷贝，避免互斥锁拷贝问题
  TrajectoryTracker(TrajectoryTracker &&other) noexcept {
    std::lock_guard<std::mutex> lock(other.state_mutex_);
    current_trajectory_ = std::move(other.current_trajectory_);
    current_point_ = std::move(other.current_point_);
    traj_start_time_ = other.traj_start_time_;
    joint_limits_ = std::move(other.joint_limits_);
    joint_space_interpolator_ = std::move(other.joint_space_interpolator_);
    is_initialized_ = other.is_initialized_.load();

    // 重置 other 对象到安全状态
    other.current_trajectory_.clear();
    other.traj_start_time_ = std::chrono::steady_clock::time_point{};
    other.is_initialized_ = false;
  }

  /**
   * @brief 更新参考轨迹
   * @param traj            参考轨迹
   */
  void SwitchReferenceTrajectory(const Trajectory &traj) {
    // corner case 1: 空轨迹
    if (traj.points.empty()) {
      L::W("trajectory tracker: empty trajectory, rejecting!");
      return;
    }
    // 如果只有一个点，调用另一个重载函数
    if (traj.points.size() == 1) {
      SwitchReferenceTrajectory(traj.points[0]);
      return;
    }
    // corner case 2: 时间不单调递增
    for (size_t i = 1; i < traj.points.size(); ++i) {
      if (Elapsed(traj.points[i]) <= Elapsed(traj.points[i - 1])) {
        L::W("trajectory tracker: non-monotonic time in trajectory, rejecting!");
        return;
      }
    }
    // corner case 3: 存在点的关节数量和对象创建时的给定数量不一致
    for (const auto &point : traj.points) {
      if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
        L::W("trajectory tracker: joint number mismatch, rejecting!");
        return;
      }
      // 检查所有位置值是否有效（非 NaN/Inf）
      for (size_t j = 0; j < NJoints; ++j) {
        if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j])) {
          L::W("trajectory tracker: invalid (NaN/Inf) values detected, rejecting!");
          return;
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_trajectory_ = traj.points;

      // 关节限位
      for (Point &point : current_trajectory_) {
        // 检查 vector 大小
        if (point.positions.size() != NJoints) {
          L::W("trajectory tracker: point size mismatch after scaling, rejecting!");
          return;
        }
        for (size_t i = 0; i < NJoints; ++i) {
          // 检查数值有效性（防止 NaN/Inf 导致的未定义行为）
          if (!std::isfinite(point.positions[i])) {
            L::W("trajectory tracker: invalid position value after scaling, rejecting!");
            return;
          }
          // 应用关节限位
          point.positions[i] =
              std::clamp(static_cast<float>(point.positions[i]), joint_limits_[i].first, joint_limits_[i].second);
        }
      }

      // 所有检查和处理通过，开始执行轨迹
      joint_space_interpolator_.SetTrajectory([&] {
        Trajectory scaled_traj;
        scaled_traj.points = current_trajectory_;
        return scaled_traj;
      }());

      // 重置轨迹开始时间（必须在同一锁内，保证原子性）
      traj_start_time_ = std::chrono::steady_clock::now();
      is_initialized_ = true;
    }
  }

  /**
   * @brief 更新一条只有一个点的参考轨迹
   * @param point 参考目标点
   */
  void SwitchReferenceTrajectory(const Point &point) {
    // corner case: 关节数量和对象创建时的给定数量不一致
    if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
      L::W("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }

    // 检查所有位置值是否有效（非 NaN/Inf）
    for (size_t j = 0; j < NJoints; ++j) {
      if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j])) {
        L::W("trajectory tracker: invalid (NaN/Inf) values detected, rejecting!");
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_trajectory_ = std::vector{point};
      // 关节限位
      for (size_t i = 0; i < NJoints; ++i) {
        current_trajectory_[0].positions[i] = std::clamp(static_cast<float>(current_trajectory_[0].positions[i]),
                                                         joint_limits_[i].first, joint_limits_[i].second);
      }
      traj_start_time_ = std::chrono::steady_clock::now();
      is_initialized_ = true;
    }
  }

  /**
   * @brief 重置轨迹（把轨迹的播放进度重置到开始）
   */
  void ResetReferenceTrajectory() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    traj_start_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief 更新一步参考点
   */
  void Update() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // corner case 1: 对象刚刚创建，没有轨迹，直接返回
    if (current_trajectory_.empty() || !is_initialized_) {
      return;
    }
    // corner case 2：轨迹里只有一个点，直接把current_point_设为这个点返回
    if (current_trajectory_.size() == 1) {
      current_point_ = current_trajectory_[0];
      return;
    }

    const auto elapsed_time = std::chrono::steady_clock::now() - traj_start_time_;

    // corner case 3：轨迹执行完毕，直接把current_point_设为最后一个点返回
    if (Elapsed(current_trajectory_.back()) < elapsed_time) {
      current_point_ = current_trajectory_[current_trajectory_.size() - 1];
      return;
    }

    // 所有检查通过，说明轨迹仍在执行中，根据已经经过的时间插值得到当前点
    current_point_ = joint_space_interpolator_(elapsed_time);

    // 对插值结果进行限位和有效性检查
    if (current_point_.positions.size() == NJoints) {
      for (size_t i = 0; i < NJoints; ++i) {
        // 检查数值有效性（防止 NaN/Inf 导致的未定义行为）
        if (!std::isfinite(current_point_.positions[i])) {
          // 如果插值结果无效，使用轨迹中的最后一个有效点
          if (!current_trajectory_.empty()) {
            current_point_ = current_trajectory_.back();
          }
          return;
        }
        // 应用关节限位
        current_point_.positions[i] = std::clamp(static_cast<float>(current_point_.positions[i]),
                                                 joint_limits_[i].first, joint_limits_[i].second);
      }
    }
  }

  std::vector<Point> trajectory() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_trajectory_;
  }
  
  Point current_point() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 返回当前安全的 current_point_（已在构造函数中初始化为各关节限位中点）
    return current_point_;
  }

  // 获取初始化状态
  bool is_initialized() const { return is_initialized_.load(); }

  // 轨迹是否执行完毕
  bool finished() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_trajectory_.empty() || !is_initialized_) {
      return true;
    }
    const auto elapsed_time = std::chrono::steady_clock::now() - traj_start_time_;
    return Elapsed(current_trajectory_.back()) < elapsed_time;
  }

 private:
  /**
   * @brief 计算轨迹点的时间戳（相对于轨迹开始）的经过时间
   * @param point 轨迹点
   * @return std::chrono::duration<int, std::milli>
   */
  static auto Elapsed(const Point &point) -> std::chrono::duration<int, std::milli> {
    return rclcpp::Duration(point.time_from_start).to_chrono<std::chrono::milliseconds>();
  }

  Point current_point_;
  std::vector<Point> current_trajectory_;
  mutable std::mutex state_mutex_;  ///< 保护所有共享状态的互斥锁
  std::chrono::steady_clock::time_point traj_start_time_;
  std::array<float, NJoints> last_joint_positions_{};
  std::array<float, NJoints> output_torque_{};
  JointSpaceInterpolator<NJoints> joint_space_interpolator_;
  std::vector<std::pair<float, float>> joint_limits_;  ///< <lower bound, upper bound>
  std::atomic_bool is_initialized_{false};             ///< 标记插值器是否已初始化
};

}  // namespace amorph