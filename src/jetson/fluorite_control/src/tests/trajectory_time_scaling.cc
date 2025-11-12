
#include <chrono>
#include <limits>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm/joint_space_interpolator.hpp"

std::chrono::duration<double> RosDurationToStdChronoDuration(const builtin_interfaces::msg::Duration &duration) {
  return std::chrono::seconds(duration.sec) + std::chrono::nanoseconds(duration.nanosec);
}
builtin_interfaces::msg::Duration StdChronoDurationToRosDuration(const std::chrono::duration<double> &duration) {
  builtin_interfaces::msg::Duration ret;
  ret.sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  const auto sec_floored = std::chrono::duration<double>(std::chrono::seconds(ret.sec));
  ret.nanosec = std::chrono::duration_cast<std::chrono::milliseconds>(duration - sec_floored).count() * 1e6;
  return ret;
}

trajectory_msgs::msg::JointTrajectory TimeScaleTrajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                                                          double playback_speed) {
  auto scaled = traj;
  playback_speed = std::clamp(playback_speed, 0.0, std::numeric_limits<double>::max());
  const auto time_ratio = 1.0 / playback_speed;
  for (auto &point : scaled.points) {
    const auto std_chrono_time_scaled = RosDurationToStdChronoDuration(point.time_from_start) * time_ratio;
    std::cout << "std_chrono_time_scaled: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(std_chrono_time_scaled).count() << std::endl;
    point.time_from_start = StdChronoDurationToRosDuration(std_chrono_time_scaled);
  }
  return scaled;
}

JointSpaceInterpolator<3> jsi{{"j1", "j2", "j3"}};

template <typename T>
void PrintVector(const std::vector<T> &vec) {
  for (const auto &elem : vec) {
    std::cout << elem << " ";
  }
  std::cout << std::endl;
}

void PrintTrajectory(const trajectory_msgs::msg::JointTrajectory &traj) {
  std::cout << "Joint Names: ";
  for (const auto &name : traj.joint_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  for (const auto &point : traj.points) {
    std::cout << "Positions: ";
    PrintVector(point.positions);
    std::cout << "Velocities: ";
    PrintVector(point.velocities);
    std::cout << "Time from start: " << point.time_from_start.sec << "." << point.time_from_start.nanosec << " seconds"
              << std::endl;
  }
}

int main() {
  const auto start = [] {
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = {0.0, 0.0, 0.0};
    start.velocities = {0.0, 0.0, 0.0};
    return start;
  }();
  const auto end = [] {
    trajectory_msgs::msg::JointTrajectoryPoint end;
    end.positions = {1.0, 1.0, 1.0};
    end.velocities = {0.0, 0.0, 0.0};
    return end;
  }();
  const auto duration = std::chrono::seconds(1);
  const auto dt = std::chrono::milliseconds(100);
  const auto traj = jsi.Interpolate(start, end, duration, dt);

  std::cout << "Interpolated Trajectory:" << std::endl;
  PrintTrajectory(traj);
  std::cout << "======================================" << std::endl;

  std::cout << "Time scaled trajectory:" << std::endl;
  PrintTrajectory(TimeScaleTrajectory(traj, 1.5));

  return 0;
}