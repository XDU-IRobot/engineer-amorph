
#ifndef ARM_PID_TRAJECTORY_TRACKER_HPP
#define ARM_PID_TRAJECTORY_TRACKER_HPP

#include <limits>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <vector>
#include <chrono>

#include <librm.hpp>

template <int NJoints>
class PidTrajectoryTracker {
  using RT = moveit_msgs::msg::RobotTrajectory;
  using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  PidTrajectoryTracker(
      std::initializer_list<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>> pids)
      : pids_(pids) {
    output_torque_.resize(NJoints);
  }
  PidTrajectoryTracker(const PidTrajectoryTracker &) = default;
  PidTrajectoryTracker(PidTrajectoryTracker &&) noexcept = default;

  /**
   * @brief 更新参考轨迹
   * @param trajectory      参考轨迹
   * @param playback_speed  参考轨迹播放速度，范围[0, std::numeric_limits<double>::max()]
   */
  void UpdateTrajectory(const RT &trajectory, double playback_speed = 1.0) {
    current_trajectory_ = trajectory.joint_trajectory.points;
    current_point_index_ = 0;
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
  }

  /**
   * @brief 更新一条只有一个点的参考轨迹
   * @param point 参考目标点
   */
  void UpdateTrajectory(const JTP &point) {
    current_trajectory_.clear();
    current_trajectory_.push_back(point);
    current_point_index_ = 0;
  }

  /**
   * @brief 重置轨迹（把轨迹的播放进度重置到开始）
   */
  void ResetReferenceTrajectory() {
    for (auto &pid : pids_) {
      pid.Clear();
    }
    traj_start_time_ = std::chrono::steady_clock::now();
    current_point_index_ = 0;
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
   * @brief 更新一步
   * @param current_joint_positions     当前关节位置，顺序和轨迹中的关节顺序一致
   * @param current_joint_velocities    当前关节速度，顺序和轨迹中的关节顺序一致
   */
  void Update(std::array<double, NJoints> current_joint_positions,
              std::array<double, NJoints> current_joint_velocities) {
    // 如果轨迹为空，直接返回
    if (current_trajectory_.empty()) {
      return;
    }

    const auto current_time = std::chrono::steady_clock::now();
    const auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_time - traj_start_time_).count();

    // 遍历轨迹，找到第一个时间戳大于当前时间的点
    // 如果没有找到，那么说明轨迹已经播放完毕，把最后一个点作为当前点
    for (size_t i = current_point_index_; i < current_trajectory_.size(); ++i) {
      current_point_index_ = i;
      if (current_trajectory_[i].time_from_start.sec * 1000 + current_trajectory_[i].time_from_start.nanosec / 1e6 >
          elapsed_time) {
        break;
      }
    }

    // 以当前点为参考计算每个关节的PID
    const auto current_point = current_trajectory_[current_point_index_];
    for (size_t i = 0; i < NJoints; ++i) {
      const auto &pid_outer = pids_[2 * i];
      const auto &pid_inner = pids_[2 * i + 1];
      pid_outer.Update(current_point.positions[i], current_joint_positions[i]);
      pid_inner.Update(current_point.velocities[i], current_joint_velocities[i]);
      output_torque_[i] = pid_inner.value();
    }
  }

  const auto &output_torque() const { return output_torque_; }

 private:
  size_t current_point_index_{};
  std::vector<JTP> current_trajectory_{};
  std::chrono::steady_clock::time_point traj_start_time_{};
  std::vector<double> output_torque_{};
  std::vector<rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>>
      pids_;  ///< outer0, inner0, outer1, inner1, ...
};

#endif