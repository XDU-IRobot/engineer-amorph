
#pragma once

#include <array>
#include <chrono>

#include <ratio>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <libInterpolate/Interpolate.hpp>

namespace amorph {

template <size_t NJoints>
class JointSpaceInterpolator {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  void SetTrajectory(const Trajectory &traj) {
    std::vector<double> x(traj.points.size());
    std::vector<double> y(traj.points.size());
    for (size_t i = 0; i < traj.points.size(); ++i) {
      x[i] = std::chrono::duration_cast<std::chrono::milliseconds>(
                 RosDurationToChrono(traj.points[i].time_from_start))
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
    ret.time_from_start = ChronoToRosDuration(time_elapsed);
    ret.velocities.resize(NJoints);
    ret.positions.resize(NJoints);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed).count();
    for (size_t i = 0; i < NJoints; ++i) {
      ret.positions[i] = interpolators_[i](elapsed_ms);
    }
    return ret;
  }

 private:
  std::chrono::duration<int, std::milli> RosDurationToChrono(const builtin_interfaces::msg::Duration &duration) {
    return rclcpp::Duration(duration).to_chrono<std::chrono::milliseconds>();
  }
  static builtin_interfaces::msg::Duration ChronoToRosDuration(
      const std::chrono::duration<double> &duration) {
       const rclcpp::Duration ros_duration(duration);
    builtin_interfaces::msg::Duration ret;
    ret.sec = ros_duration.seconds();
    ret.nanosec = ros_duration.nanoseconds() % 1000000000;
    return ret;
  }

  std::array<_1D::CubicSplineInterpolator<double>, NJoints> interpolators_;
};

} // namespace amorph