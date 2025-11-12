
#include "arm/joint_space_interpolator.hpp"

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

  return 0;
}