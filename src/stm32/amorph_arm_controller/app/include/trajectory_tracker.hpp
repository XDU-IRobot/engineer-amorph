#pragma once

#include <vector>
#include <chrono>
#include <utility>
#include <initializer_list>

#include <librm.hpp>

#include "joint_space_interpolator.hpp"
#include "rpc/rpc.hpp"

using namespace rm::modules::angle_literals;

/**
 * @brief   用来沿时间进度“播放”一条Multi-dof轨迹的Tracker，命名灵感来自那种老电脑上做音乐的Tracker :))
 * @tparam  NJoints 关节数量
 */
template <int NJoints>
class TrajectoryTracker {
  using Trajectory = JointTrajectory;
  using Point = JointTrajectoryPoint;
  using Duration = std::chrono::duration<int, std::milli>;

 public:
  explicit TrajectoryTracker(std::initializer_list<std::pair<float, float>> joint_limits)
      : joint_limits_(joint_limits), is_initialized_(false) {
    if (joint_limits_.size() != NJoints) {
      // globals->rpc.Fatal("trajectory tracker: number of pairs of joint limit mismatch! expected {}, got {}", NJoints,
      //                    joint_limits_.size());
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
  TrajectoryTracker(const TrajectoryTracker &) = default;
  TrajectoryTracker(TrajectoryTracker &&other) noexcept {
    current_trajectory_ = std::move(other.current_trajectory_);
    current_point_ = std::move(other.current_point_);
    traj_start_time_ = other.traj_start_time_;
    joint_limits_ = std::move(other.joint_limits_);
    joint_space_interpolator_ = std::move(other.joint_space_interpolator_);
    is_initialized_ = other.is_initialized_;

    // 重置 other 对象到安全状态
    other.current_trajectory_.clear();
    other.traj_start_time_ = std::chrono::steady_clock::time_point{};
    other.is_initialized_ = false;
  }

  /**
   * @brief 更新参考轨迹
   * @param traj            参考轨迹
   * @param playback_speed  参考轨迹播放速度，范围[0, std::numeric_limits<double>::max()]
   * @note  速度和加速度安全检查由上层轨迹限制器负责，此类只负责关节限位
   */
  void SwitchReferenceTrajectory(const Trajectory &traj, double playback_speed = 1.0) {
    // corner case 1: 空轨迹
    if (traj.points.empty()) {
      // logging::warn("trajectory tracker: empty trajectory, rejecting!");
      return;
    }
    // 如果只有一个点，调用另一个重载函数
    if (traj.points.size() == 1) {
      SwitchReferenceTrajectory(traj.points[0]);
      return;
    }
    // corner case 2: 时间不单调递增
    for (size_t i = 1; i < traj.points.size(); ++i) {
      if (PointTimeMs(traj.points[i]) <= PointTimeMs(traj.points[i - 1])) {
        // logging::warn("trajectory tracker: non-monotonic time in trajectory, rejecting!");
        return;
      }
    }
    // corner case 3: 存在点的关节数量和对象创建时的给定数量不一致
    for (const auto &point : traj.points) {
      if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
        // logging::warn("trajectory tracker: joint number mismatch, rejecting!");
        return;
      }
      // 检查所有位置值是否有效（非 NaN/Inf）
      for (size_t j = 0; j < NJoints; ++j) {
        if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j])) {
          // logging::warn("trajectory tracker: invalid (NaN/Inf) values detected, rejecting!");
          return;
        }
      }
    }

    Trajectory traj_processed = traj;
    // 额外处理：如果第一个点的时间戳不等于0，则从当前位置开始计算一段插值轨迹，平滑地到达这个点
    if (PointTimeMs(traj.points[0]) != 0) {
      const auto duration = traj.points[0].time_from_start;
      const auto dt = std::chrono::duration<int, std::milli>(50);
      const Point start_point = current_point_;
      const Point end_point = traj.points[0];
      const auto interpolated_trajectory = joint_space_interpolator_.Interpolate(start_point, end_point, duration, dt);

      // 验证插值结果
      if (interpolated_trajectory.points.empty()) {
        // logging::warn("trajectory tracker: interpolation failed, rejecting!");
        return;
      }

      // 把计算出来的这段插值轨迹插到已有的轨迹前面
      traj_processed.points.insert(traj_processed.points.begin(), interpolated_trajectory.points.begin(),
                                   interpolated_trajectory.points.end());
      // logging::info("trajectory tracker: added interpolated trajectory to the beginning of the trajectory.");
    }

    // 所有检查通过，开始执行轨迹
    {
      // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
      current_trajectory_ =
          TimeScale(traj_processed, std::clamp(playback_speed, 1e-3, std::numeric_limits<double>::max())).points;

      // 验证 TimeScale 结果
      if (current_trajectory_.empty()) {
        // logging::warn("trajectory tracker: time scaling resulted in empty trajectory, rejecting!");
        return;
      }

      // 关节限位
      for (Point &point : current_trajectory_) {
        // 检查 vector 大小
        if (point.positions.size() != NJoints) {
          // logging::warn("trajectory tracker: point size mismatch after scaling, rejecting!");
          return;
        }
        for (size_t i = 0; i < NJoints; ++i) {
          // 检查数值有效性（防止 NaN/Inf 导致的未定义行为）
          if (!std::isfinite(point.positions[i])) {
            // logging::warn("trajectory tracker: invalid position value after scaling, rejecting!");
            return;
          }
          // 应用关节限位
          point.positions[i] = std::clamp(point.positions[i], joint_limits_[i].first, joint_limits_[i].second);
        }
      }

      joint_space_interpolator_.SetTrajectory([&] {
        Trajectory scaled_traj;
        scaled_traj.points = current_trajectory_;
        return scaled_traj;
      }());

      is_initialized_ = true;
    }
    ResetReferenceTrajectory();
  }

  /**
   * @brief 更新一条只有一个点的参考轨迹
   * @param point 参考目标点
   */
  void SwitchReferenceTrajectory(const Point &point) {
    // corner case: 关节数量和对象创建时的给定数量不一致
    if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
      // logging::warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }

    // 检查所有位置值是否有效（非 NaN/Inf）
    for (size_t j = 0; j < NJoints; ++j) {
      if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j])) {
        // logging::warn("trajectory tracker: invalid (NaN/Inf) values detected, rejecting!");
        return;
      }
    }

    {
      // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
      current_trajectory_ = std::vector{point};
      // 关节限位
      for (size_t i = 0; i < NJoints; ++i) {
        current_trajectory_[0].positions[i] =
            std::clamp(current_trajectory_[0].positions[i], joint_limits_[i].first, joint_limits_[i].second);
      }
      traj_start_time_ = std::chrono::steady_clock::now();
      is_initialized_ = true;
    }
    ResetReferenceTrajectory();
  }

  /**
   * @brief 重置轨迹（把轨迹的播放进度重置到开始）
   */
  void ResetReferenceTrajectory() { traj_start_time_ = std::chrono::steady_clock::now(); }

  /**
   * @brief 更新一步参考点
   */
  void Update() {
    // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
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
    if (current_trajectory_[current_trajectory_.size() - 1].time_from_start < elapsed_time) {
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
        current_point_.positions[i] =
            std::clamp(current_point_.positions[i], joint_limits_[i].first, joint_limits_[i].second);
      }
    }
  }

  const auto &trajectory() const { return current_trajectory_; }
  const auto &current_point() const {
    // Update();
    if (current_trajectory_.empty() || !is_initialized_) {
      // 返回当前安全的 current_point_（已在构造函数中初始化为各关节限位中点）
      return current_point_;
    }
    return current_point_;
  }

  // 获取初始化状态
  bool is_initialized() const { return is_initialized_; }

  // 轨迹是否执行完毕
  bool IsTrajectoryFinished() const {
    if (current_trajectory_.empty() || !is_initialized_) {
      return true;
    }
    const auto elapsed_time = std::chrono::steady_clock::now() - traj_start_time_;
    return current_trajectory_.back().time_from_start < elapsed_time;
  }

 private:
  static int64_t PointTimeMs(const Point &point) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(point.time_from_start).count();
  }
  static Trajectory TimeScale(const Trajectory &traj, float playback_speed) {
    Trajectory scaled{traj};
    playback_speed = std::clamp(playback_speed, 0.0f, std::numeric_limits<float>::max());
    for (auto &point : scaled.points) {
      point.time_from_start = std::chrono::duration_cast<Duration>(point.time_from_start / playback_speed);
    }
    return scaled;
  }

  Point current_point_{};  // 在构造函数中初始化
  std::vector<Point> current_trajectory_{};
  // std::mutex current_trajectory_mutex_{};
  std::chrono::steady_clock::time_point traj_start_time_{};
  std::array<float, NJoints> last_joint_positions_{};
  std::array<float, NJoints> output_torque_{};
  JointSpaceInterpolator<NJoints> joint_space_interpolator_;
  std::vector<std::pair<float, float>> joint_limits_;  ///< <lower bound, upper bound>
  bool is_initialized_;                                ///< 标记插值器是否已初始化
};
