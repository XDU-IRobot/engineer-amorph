
#pragma once

#include <vector>
#include <iostream>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

template <typename T>
void Print(const std::vector<T> &vec) {
  std::cout << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    std::cout << vec[i];
    if (i != vec.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

inline void Print(const trajectory_msgs::msg::JointTrajectory &traj) {
  std::cout << "JointTrajectory:" << std::endl;
  std::cout << "  header:" << std::endl;
  std::cout << "    stamp: " << traj.header.stamp.sec << "." << traj.header.stamp.nanosec << std::endl;
  std::cout << "    frame_id: " << traj.header.frame_id << std::endl;
  std::cout << "  joint_names: ";
  Print(traj.joint_names);
  std::cout << "  points:" << std::endl;
  for (const auto &point : traj.points) {
    std::cout << "    time_from_start: " << point.time_from_start.sec << "." << point.time_from_start.nanosec
              << std::endl;
    std::cout << "    positions: ";
    Print(point.positions);
    std::cout << "    velocities: ";
    Print(point.velocities);
    std::cout << "    accelerations: ";
    Print(point.accelerations);
    std::cout << "    effort: ";
    Print(point.effort);
  }
}

inline void Print(const trajectory_msgs::msg::JointTrajectoryPoint &point) {
  std::cout << "JointTrajectoryPoint:" << std::endl;
  std::cout << "  time_from_start: " << point.time_from_start.sec << "." << point.time_from_start.nanosec << std::endl;
  std::cout << "  positions: ";
  Print(point.positions);
  std::cout << "  velocities: ";
  Print(point.velocities);
  std::cout << "  accelerations: ";
  Print(point.accelerations);
  std::cout << "  effort: ";
  Print(point.effort);
}