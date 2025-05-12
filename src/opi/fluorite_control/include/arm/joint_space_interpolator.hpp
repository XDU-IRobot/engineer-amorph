
#ifndef ARM_JOINT_SPACE_INTERPOLATOR_HPP
#define ARM_JOINT_SPACE_INTERPOLATOR_HPP

#include <array>
#include <chrono>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <libInterpolate/Interpolate.hpp>

// TODO: WIPWIPWIPWIP!!!DONT USE
template <size_t NJoints>
class JointSpaceInterpolator {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

 public:
  JointSpaceInterpolator() = delete;
  explicit JointSpaceInterpolator(const std::array<std::string, NJoints> &joint_names) : joint_names_(joint_names) {}
  Trajectory Interpolate(const Point &start, const Point &end, std::chrono::duration<double> duration,
                         std::chrono::duration<double> dt) {
    Trajectory traj;
    const size_t num_points =
        static_cast<size_t>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() /
                            std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
    traj.points.resize(num_points);
    traj.joint_names = joint_names_;
    for (size_t i = 0; i < num_points; ++i) {
      for (size_t j = 0; j < NJoints; ++j) {
        const std::vector x = {0.0, static_cast<double>(num_points - 1)};
        const std::vector y = {start.positions[j], end.positions[j]};
        interpolators_[j].setData(x, y);
      }
      traj.points[i].positions.resize(NJoints);
      traj.points[i].velocities.resize(NJoints);
    }
    return traj;
  }

 private:
  auto StdChronoDurationToRosDuration(const std::chrono::duration<double> &duration) {
    builtin_interfaces::msg::Duration ret;
    ret.sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
    ret.nanosec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() -
                                        std::chrono::duration_cast<std::chrono::seconds>(duration).count() * 1e9);
    return ret;
  }

  std::array<std::string, NJoints> joint_names_;
  std::array<_1D::CubicSplineInterpolator<double>, NJoints> interpolators_;
};

#endif  // ARM_JOINT_SPACE_INTERPOLATOR_HPP