
#ifndef ARM_JOINT_SPACE_INTERPOLATOR_HPP
#define ARM_JOINT_SPACE_INTERPOLATOR_HPP

#include <array>
#include <chrono>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <libInterpolate/Interpolate.hpp>
#include "common/debug_helpers.hpp"

template <size_t NJoints>
class JointSpaceInterpolator {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  JointSpaceInterpolator() = delete;
  explicit JointSpaceInterpolator(std::vector<std::string> joint_names) : joint_names_(std::move(joint_names)) {
    // if (joint_names.size() != NJoints) {
    //   throw std::invalid_argument("Joint names size mismatch!");
    // }
  }
  Trajectory Interpolate(const Point &start, const Point &end, std::chrono::duration<double> duration,
                         std::chrono::duration<double> dt) {
    Trajectory traj;
    const size_t num_points =
        static_cast<size_t>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() /
                            std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
    traj.points.resize(num_points);
    traj.joint_names = joint_names_;
    for (size_t i = 0; i < NJoints; ++i) {
      const std::vector x = {0.0, static_cast<double>(num_points - 1)};
      const std::vector y = {start.positions[i], end.positions[i]};
      interpolators_[i].setData(x, y);
    }
    for (size_t i = 0; i < num_points; ++i) {
      traj.points[i].velocities.resize(NJoints);
      traj.points[i].positions.resize(NJoints);
      traj.points[i].time_from_start = StdChronoDurationToRosDuration(dt * i);
      for (size_t j = 0; j < NJoints; ++j) {
        traj.points[i].positions[j] = interpolators_[j](i);
      }
    }
    return traj;
  }

  void SetTrajectory(const Trajectory &traj) {
    std::vector<double> x(traj.points.size());
    std::vector<double> y(traj.points.size());
    for (size_t i = 0; i < traj.points.size(); ++i) {
      x[i] = std::chrono::duration_cast<std::chrono::milliseconds>(
                 RosDurationToStdChronoDuration(traj.points[i].time_from_start))
                 .count();
    }
    for (size_t i = 0; i < NJoints; ++i) {
      for (size_t j = 0; j < traj.points.size(); ++j) {
        y[j] = traj.points[j].positions[i];
      }
      interpolators_[i].setData(x, y);
    }
  }

  /**
   * @param time_elapsed duration没有做边界检查，调用时注意不要超出轨迹范围
   */
  Point operator()(const std::chrono::duration<double> &time_elapsed) {
    Point ret;
    ret.time_from_start = StdChronoDurationToRosDuration(time_elapsed);
    ret.velocities.resize(NJoints);
    ret.positions.resize(NJoints);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed).count();
    for (size_t i = 0; i < NJoints; ++i) {
      ret.positions[i] = interpolators_[i](elapsed_ms);
    }
    return ret;
  }

 private:
  std::chrono::duration<double> RosDurationToStdChronoDuration(const builtin_interfaces::msg::Duration &duration) {
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

  std::vector<std::string> joint_names_;
  std::array<_1D::CubicSplineInterpolator<double>, NJoints> interpolators_;
};

#endif  // ARM_JOINT_SPACE_INTERPOLATOR_HPP