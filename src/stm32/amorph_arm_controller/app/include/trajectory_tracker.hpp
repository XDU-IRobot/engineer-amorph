
#pragma once

#include <vector>
#include <chrono>
// #include <mutex>
#include <utility>
#include <initializer_list>

#include <librm.hpp>

#include "joint_space_interpolator.hpp"
#include "rpc/rpc.hpp"

namespace fluorite::arm {

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
      : joint_limits_(joint_limits) {
    if (joint_limits_.size() != NJoints) {
      globals->rpc.Fatal("trajectory tracker: number of pairs of joint limit mismatch! expected {}, got {}", NJoints,
                     joint_limits_.size());
    }
  }
  TrajectoryTracker(const TrajectoryTracker &) = default;
  TrajectoryTracker(TrajectoryTracker &&other) noexcept {
    current_trajectory_ = std::move(other.current_trajectory_);
    traj_start_time_ = other.traj_start_time_;
    joint_limits_ = std::move(other.joint_limits_);
  }

  /**
   * @brief 更新参考轨迹
   * @param trajectory      参考轨迹
   * @param playback_speed  参考轨迹播放速度，范围[0, std::numeric_limits<double>::max()]
   */
  void SwitchReferenceTrajectory(const Trajectory &traj, double playback_speed = 1.0, bool forced = false) {
    // corner case 1: 空轨迹
    if (traj.points.empty()) {
      logging::warn("trajectory tracker: empty trajectory, rejecting!");
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
        logging::warn("trajectory tracker: non-monotonic time in trajectory, rejecting!");
        return;
      }
    }
    // corner case 3: 存在点的关节数量和对象创建时的给定数量不一致
    for (const auto &point : traj.points) {
      if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
        logging::warn("trajectory tracker: joint number mismatch, rejecting!");
        return;
      }
    }
    // corner case 4: 存在某相邻两点之间至少一个关节的速度太大
    if (!forced) {
      for (size_t i = 1; i < traj.points.size(); ++i) {
        const auto &point1 = traj.points[i - 1];
        const auto &point2 = traj.points[i];
        constexpr double kMaxVelocity = M_PI * 1.3;
        const double dt = (PointTimeMs(point2) - PointTimeMs(point1)) * 1e-3;  // ms to s
        double max_vel = 0.0;
        for (size_t j = 0; j < NJoints; ++j) {
          const auto velocity = std::abs(point2.positions[j] - point1.positions[j]) / dt;
          if (velocity > max_vel) {
            max_vel = velocity;
          }
          if (velocity > kMaxVelocity) {
            logging::warn("trajectory tracker: too large velocity in trajectory, rejecting! max_vel = {}", max_vel);
            return;
          }
        }
      }
      // corner case 4.5: 或从当前位置到轨迹第一个点的速度过大
      if (PointTimeMs(traj.points[0]) == 0) {
        // 防止除零，如果第一个点的时间戳是0，则直接判断当前位置到这个点的角度差
        // 不允许某个关节的角度差大于30度
        for (size_t j = 0; j < NJoints; ++j) {
          const auto position_delta = std::abs(current_point_.positions[j] - traj.points[0].positions[j]);
          if (position_delta > (30.0_deg).rad()) {
            logging::warn("trajectory tracker: no teleport, rejecting!");
            return;
          }
        }
      }
      const auto &point1 = traj.points[0];
      const auto &point2 = current_point_;
      constexpr double kMaxVelocity = M_PI * 1.3;
      const double dt = (PointTimeMs(point1) - PointTimeMs(point2)) * 1e-3;  // ms to s
      double max_vel = 0.0;
      for (size_t j = 0; j < NJoints; ++j) {
        const auto velocity = std::abs(point1.positions[j] - point2.positions[j]) / dt;
        if (velocity > max_vel) {
          max_vel = velocity;
        }
        if (velocity > kMaxVelocity) {
          logging::warn("trajectory tracker: too large velocity to reach the first point, rejecting! max_vel = %f",
                        max_vel);
          return;
        }
      }
    }

    Trajectory traj_processed = traj;
    // 额外处理：如果第一个点的时间戳不等于0，则从当前位置开始计算一段插值轨迹，平滑地到达这个点
    if (PointTimeMs(traj.points[0]) != 0) {
      const auto duration = traj.points[0].time_from_start;
      const auto dt = std::chrono::milliseconds(50);
      const Point start_point = current_point_;
      const Point end_point = traj.points[0];
      const auto interpolated_trajectory = joint_space_interpolator_.Interpolate(start_point, end_point, duration, dt);

      // 把计算出来的这段插值轨迹插到已有的轨迹前面
      traj_processed.points.insert(traj_processed.points.begin(), interpolated_trajectory.points.begin(),
                                   interpolated_trajectory.points.end());
      logging::info("trajectory tracker: added interpolated trajectory to the beginning of the trajectory.");
    }

    // 所有检查通过，开始执行轨迹
    {
      // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
      current_trajectory_ =
          TimeScale(traj_processed, std::clamp(playback_speed, 1e-3, std::numeric_limits<double>::max())).points;
      // 限位
      for (Point &point : current_trajectory_) {
        for (size_t i = 0; i < NJoints; ++i) {
          point.positions[i] = std::clamp(point.positions[i], joint_limits_[i].first, joint_limits_[i].second);
        }
      }

      joint_space_interpolator_.SetTrajectory([&] {
        Trajectory scaled_traj;
        scaled_traj.points = current_trajectory_;
        return scaled_traj;
      }());
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
      logging::warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }
    {
      // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
      current_trajectory_ = std::vector{point};
      // 限位
      for (size_t i = 0; i < NJoints; ++i) {
        current_trajectory_[0].positions[i] =
            std::clamp(current_trajectory_[0].positions[i], joint_limits_[i].first, joint_limits_[i].second);
      }
      traj_start_time_ = std::chrono::steady_clock::now();
    }
    ResetReferenceTrajectory();
  }

  /**
   * @brief 从当前位置样条插值到一个目标点
   * @param point 参考目标点
   * @param duration 用多少时间到达这个点
   * @warning BROKEN DONT USE
   */
  [[deprecated]] void SwitchReferenceTrajectory(const Point &point, Duration duration,
                                                Duration dt = std::chrono::milliseconds(50)) {
    // corner case: 关节数量和对象创建时的给定数量不一致
    if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
      logging::warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }
    // corner case: Duration < 0
    if (duration.count() < 0) {
      logging::warn("trajectory tracker: negative duration, rejecting!");
      return;
    }
    Update();
    {
      // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);

      if (std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > 0.1) {
        const Trajectory interpolated_trajectory =
            joint_space_interpolator_.Interpolate(current_point_, point, duration, dt);
        current_trajectory_ = interpolated_trajectory.points;
      } else {
        current_trajectory_ = {point};  // 如果duration太小，就不插值了，直接用这个点
      }

      // 限位
      for (size_t i = 0; i < NJoints; ++i) {
        current_trajectory_[0].positions[i] =
            std::clamp(current_trajectory_[0].positions[i], joint_limits_[i].first, joint_limits_[i].second);
      }
      traj_start_time_ = std::chrono::steady_clock::now();
    }
    ResetReferenceTrajectory();
  }

  /**
   * @brief 重置轨迹（把轨迹的播放进度重置到开始）
   */
  void ResetReferenceTrajectory() { traj_start_time_ = std::chrono::steady_clock::now(); }

  /**
   * @brief 更新一步参考点，但不计算PID
   */
  void Update() {
    // std::lock_guard<std::mutex> lock(current_trajectory_mutex_);

    // corner case 1: 对象刚刚创建，没有轨迹，直接返回
    if (current_trajectory_.empty()) {
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
  }

  const auto &trajectory() const { return current_trajectory_; }
  const auto &current_point() const {
    // Update();
    if (current_trajectory_.empty()) {
      const static auto kEmptyPoint = [] {
        Point point;
        point.positions.resize(NJoints);
        point.velocities.resize(NJoints);
        point.accelerations.resize(NJoints);
        point.effort.resize(NJoints);
        return point;
      }();
      return kEmptyPoint;
    }
    return current_point_;
  }

 private:
  int64_t PointTimeMs(const Point &point) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(point.time_from_start).count();
  }
  static Trajectory TimeScale(const Trajectory &traj, float playback_speed) {
    Trajectory scaled{traj};
    playback_speed = std::clamp(playback_speed, 0.0f, std::numeric_limits<float>::max());
    for (auto &point : scaled.points) {
      // TODO: point.time_from_start = point.time_from_start * (1.0f / playback_speed);
    }
    return scaled;
  }

  Point current_point_{[] {
    Point p;
    p.velocities.resize(NJoints);
    p.positions.resize(NJoints);
    return p;
  }()};
  std::vector<Point> current_trajectory_{};
  // std::mutex current_trajectory_mutex_{};
  std::chrono::steady_clock::time_point traj_start_time_{};
  std::array<float, NJoints> last_joint_positions_{};
  std::array<float, NJoints> output_torque_{};
  JointSpaceInterpolator<NJoints> joint_space_interpolator_;
  std::vector<std::pair<float, float>> joint_limits_;  ///< <lower bound, upper bound>
};

}  // namespace fluorite::arm