
#ifndef ARM_PID_TRAJECTORY_TRACKER_HPP
#define ARM_PID_TRAJECTORY_TRACKER_HPP

#include <array>
#include <limits>
#include <vector>
#include <chrono>
#include <mutex>
#include <utility>
#include <initializer_list>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <librm.hpp>

#include "arm/joint_space_interpolator.hpp"
#include "common/logger.hpp"
#include "common/debug_helpers.hpp"

namespace fluorite {

template <int NJoints>
class PidTrajectoryTracker {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  PidTrajectoryTracker(
      std::initializer_list<std::shared_ptr<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>>>
          pids,
      std::initializer_list<std::pair<double, double>> joint_limits)
      : pids_(pids), joint_limits_(joint_limits) {
    if (pids_.size() != 2 * NJoints) {
      fatal("trajectory tracker: PID number mismatch! expected %d, got %ld", 2 * NJoints, pids_.size());
      throw std::invalid_argument("PID number mismatch!");
    }
    if (joint_limits_.size() != NJoints) {
      fatal("trajectory tracker: number of pairs of joint limit mismatch! expected %d, got %ld", NJoints,
            joint_limits_.size());
      throw std::invalid_argument("number of pairs of joint limit mismatch!");
    }

    std::printf("=======================================================\n");
    std::printf("trajectory tracker:\n");
    for (size_t i = 0; i < 2 * NJoints; ++i) {
      std::printf("PID %zu: P=%f, I=%f, D=%f, max_iout=%f, max_out=%f\n", i, pids_[i]->kp_, pids_[i]->ki_,
                  pids_[i]->kd_, pids_[i]->max_iout_, pids_[i]->max_out_);
    }
    std::printf("=======================================================\n");
  }
  PidTrajectoryTracker(const PidTrajectoryTracker &) = default;
  PidTrajectoryTracker(PidTrajectoryTracker &&other) noexcept {
    std::lock_guard<std::mutex> lock(other.current_trajectory_mutex_);
    current_trajectory_ = std::move(other.current_trajectory_);
    traj_start_time_ = other.traj_start_time_;
    pids_ = std::move(other.pids_);
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
      warn("trajectory tracker: empty trajectory, rejecting!");
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
    // corner case 4: 存在某相邻两点之间至少一个关节的速度太大
    if (!forced) {
      for (size_t i = 1; i < traj.points.size(); ++i) {
        const auto &point1 = traj.points[i - 1];
        const auto &point2 = traj.points[i];
        constexpr double kMaxVelocity = 90.0 / 180.0 * M_PI;                   // 90度/s
        const double dt = (PointTimeMs(point2) - PointTimeMs(point1)) * 1e-3;  // ms to s
        for (size_t j = 0; j < NJoints; ++j) {
          const auto velocity = std::abs(point2.positions[j] - point1.positions[j]) / dt;
          if (velocity > kMaxVelocity) {
            warn("trajectory tracker: too large velocity in trajectory, rejecting!");
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
          if (position_delta > 30.0 / 180.0 * M_PI) {
            warn("trajectory tracker: no teleport, rejecting!");
            return;
          }
        }
      }
      const auto &point1 = traj.points[0];
      const auto &point2 = current_point_;
      constexpr double kMaxVelocity = 90.0 / 180.0 * M_PI;                   // 90度/s
      const double dt = (PointTimeMs(point1) - PointTimeMs(point2)) * 1e-3;  // ms to s
      for (size_t j = 0; j < NJoints; ++j) {
        const auto velocity = std::abs(point1.positions[j] - point2.positions[j]) / dt;
        if (velocity > kMaxVelocity) {
          warn("trajectory tracker: too large velocity in trajectory, rejecting!");
          return;
        }
      }
    }

    Trajectory traj_processed = traj;
    // 额外处理：如果第一个点的时间戳不等于0，则从当前位置开始计算一段插值轨迹，平滑地到达这个点
    if (PointTimeMs(traj.points[0]) != 0) {
      const auto duration = RosDurationToStdChronoDuration(traj.points[0].time_from_start);
      const auto dt = std::chrono::milliseconds(50);
      const Point start_point = current_point_;
      const Point end_point = traj.points[0];
      const auto interpolated_trajectory = joint_space_interpolator_.Interpolate(start_point, end_point, duration, dt);

      // 把计算出来的这段插值轨迹插到已有的轨迹前面
      traj_processed.points.insert(traj_processed.points.begin(), interpolated_trajectory.points.begin(),
                                   interpolated_trajectory.points.end());
      info("trajectory tracker: added interpolated trajectory to the beginning of the trajectory.");
    }

    // 所有检查通过，开始执行轨迹
    {
      std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
      current_trajectory_ =
          TimeScaleTrajectory(traj_processed, std::clamp(playback_speed, 1e-3, std::numeric_limits<double>::max()))
              .points;
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
      warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(current_trajectory_mutex_);
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
  [[deprecated]] void SwitchReferenceTrajectory(const Point &point, std::chrono::duration<double> duration,
                                                std::chrono::duration<double> dt = std::chrono::milliseconds(50)) {
    // corner case: 关节数量和对象创建时的给定数量不一致
    if (point.positions.size() != NJoints || point.velocities.size() != NJoints) {
      warn("trajectory tracker: joint number mismatch, rejecting!");
      return;
    }
    // corner case: duration < 0
    if (duration.count() < 0) {
      warn("trajectory tracker: negative duration, rejecting!");
      return;
    }
    Update();
    {
      std::lock_guard<std::mutex> lock(current_trajectory_mutex_);

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
   * @brief 重置所有pid控制器的内部状态
   */
  void ClearPid() {
    for (auto &pid : pids_) {
      pid->Clear();
    }
  }

  /**
   * @brief 更新一步参考点，但不计算PID
   */
  void Update() {
    std::lock_guard<std::mutex> lock(current_trajectory_mutex_);

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
    if (RosDurationToStdChronoDuration(current_trajectory_[current_trajectory_.size() - 1].time_from_start) <
        elapsed_time) {
      current_point_ = current_trajectory_[current_trajectory_.size() - 1];
      return;
    }

    // 所有检查通过，说明轨迹仍在执行中，根据已经经过的时间插值得到当前点
    current_point_ = joint_space_interpolator_(elapsed_time);
  }

  /**
   * @brief 更新一步
   * @param current_joint_positions     当前关节位置，顺序和轨迹中的关节顺序一致
   * @param current_joint_velocities    当前关节速度，顺序和轨迹中的关节顺序一致
   */
  void Update(std::array<double, NJoints> current_joint_positions,
              std::array<double, NJoints> current_joint_velocities) {
    // corner case: 对象刚刚创建，没有轨迹，把输入的当前位置作为目标位置
    if (current_trajectory_.empty()) {
      Point point;
      point.positions.resize(NJoints);
      for (size_t i = 0; i < NJoints; ++i) {
        point.positions[i] = current_joint_positions[i];
      }
      point.velocities.resize(NJoints);
      SwitchReferenceTrajectory(point);
      warn("trajectory tracker: no reference trajectory, using current position as reference.");
    }
    Update();

    // 以当前点为参考计算每个关节的PID
    for (size_t i = 0; i < NJoints; ++i) {
      auto &pid_outer = pids_[2 * i];
      auto &pid_inner = pids_[2 * i + 1];
      pid_inner->Update(current_point_.positions[i], current_joint_positions[i]);
      pid_outer->Update(pid_inner->value(), current_joint_velocities[i]);
      output_torque_[i] = pid_outer->value();

      // 并且把这一次的关节位置存起来，用来判断轨迹有没有执行成功
      last_joint_positions_[i] = current_joint_positions[i];
    }
  }

  bool execution_finished() {
    if (current_trajectory_.empty()) {
      return true;
    }
    constexpr double kDiffThreshold = 13.5 / 360.0 * 2.0 * M_PI;  // 4度
    const auto &last_point = current_trajectory_[current_trajectory_.size() - 1];
    // 计算上一次传入的关节位置和目标轨迹最后一个点的位置的差值，如果所有关节的误差都小于阈值，则认为执行完毕
    for (size_t i = 0; i < NJoints; ++i) {
      if (std::abs(last_joint_positions_[i] - last_point.positions[i]) > kDiffThreshold) {
        return false;
      }
    }
    return true;
  }
  const auto &pids() const { return pids_; }
  const auto &output_torque() const { return output_torque_; }
  const auto &trajoctory() const { return current_trajectory_; }
  const auto &current_point() {
    const static auto kEmptyPoint = [] {
      Point point;
      point.positions.resize(NJoints);
      point.velocities.resize(NJoints);
      point.accelerations.resize(NJoints);
      point.effort.resize(NJoints);
      return point;
    }();
    Update();
    if (current_trajectory_.empty()) {
      return kEmptyPoint;
    }
    return current_point_;
  }

 private:
  int64_t PointTimeMs(const Point &point) const {
    return point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1e6;
  }
  static std::chrono::duration<double> RosDurationToStdChronoDuration(
      const builtin_interfaces::msg::Duration &duration) {
    return std::chrono::seconds(duration.sec) + std::chrono::nanoseconds(duration.nanosec);
  }
  static builtin_interfaces::msg::Duration StdChronoDurationToRosDuration(
      const std::chrono::duration<double> &duration) {
    builtin_interfaces::msg::Duration ret;
    ret.sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
    const auto sec_floored = std::chrono::duration<double>(std::chrono::seconds(ret.sec));
    ret.nanosec = std::chrono::duration_cast<std::chrono::milliseconds>(duration - sec_floored).count() * 1e6;
    return ret;
  }
  static Trajectory TimeScaleTrajectory(const Trajectory &traj, double playback_speed) {
    auto scaled = traj;
    playback_speed = std::clamp(playback_speed, 0.0, std::numeric_limits<double>::max());
    const auto time_ratio = 1.0 / playback_speed;
    for (auto &point : scaled.points) {
      const auto std_chrono_time_scaled = RosDurationToStdChronoDuration(point.time_from_start) * time_ratio;
      point.time_from_start = StdChronoDurationToRosDuration(std_chrono_time_scaled);
    }
    return scaled;
  }

  Point current_point_{[] {
    Point p;
    p.velocities.resize(6);
    p.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return p;
  }()};
  std::vector<Point> current_trajectory_{};
  std::mutex current_trajectory_mutex_{};
  std::chrono::steady_clock::time_point traj_start_time_{};
  std::array<double, NJoints> last_joint_positions_{};
  std::array<double, NJoints> output_torque_{};
  JointSpaceInterpolator<NJoints> joint_space_interpolator_{
      {"yaw", "pitch_1", "pitch_2", "end_yaw_1", "end_pitch", "end_yaw_2"}};
  std::vector<std::shared_ptr<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>>>
      pids_;                                             ///< outer0, inner0, outer1, inner1, ...
  std::vector<std::pair<double, double>> joint_limits_;  ///< <lower bound, upper bound>
};

}  // namespace fluorite

#endif