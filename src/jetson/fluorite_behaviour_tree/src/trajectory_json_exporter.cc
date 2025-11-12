#include <chrono>
#include <string>
#include <iostream>

#include "moveit.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

template <size_t ND>
struct Point : trajectory_msgs::msg::JointTrajectoryPoint {
  Point() : trajectory_msgs::msg::JointTrajectoryPoint() {
    positions.resize(ND);
    velocities.resize(ND);
    accelerations.resize(ND);
    effort.resize(ND);
  }
};

int main() {
  using namespace std::chrono_literals;

  Point<6> a, b;

  a.time_from_start = rclcpp::Duration(1000ms);
  a.positions = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0};
  b.time_from_start = rclcpp::Duration(1250ms);
  b.positions = {-2.75, 1.74, 1.9, 0.015, -1.743, -0.104};

  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = {"yaw_joint",       "pitch_1_joint",   "pitch_2_joint",
                   "end_yaw_1_joint", "end_pitch_joint", "end_yaw_2_joint"};
  t.points = {a};

  std::string json_str = JsonBuilder::BuildJson(t);
  std::cout << json_str << std::endl;

  std::cout << "==== Deserialize back test ====" << std::endl;
  auto t2 = BT::convertFromString<trajectory_msgs::msg::JointTrajectory>(json_str);

  for (size_t i = 0; i < t2.points.size(); ++i) {
    const auto& pt = t2.points[i];
    std::cout << "Point " << i << ":" << std::endl;
    std::cout << "  time_from_start: " << pt.time_from_start.sec << "s " << pt.time_from_start.nanosec << "ns"
              << std::endl;
    std::cout << "  positions: ";
    for (auto v : pt.positions) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "  velocities: ";
    for (auto v : pt.velocities) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "  accelerations: ";
    for (auto v : pt.accelerations) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "  effort: ";
    for (auto v : pt.effort) std::cout << v << " ";
    std::cout << std::endl;
  }

  return 0;
}