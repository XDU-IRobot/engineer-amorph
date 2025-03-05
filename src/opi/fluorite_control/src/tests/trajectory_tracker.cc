
#include "arm/pid_trajectory_tracker.hpp"
#include "common/logger.hpp"

#include <chrono>
#include <iostream>

std::shared_ptr<moveit_msgs::msg::RobotTrajectory> GenerateValidTrajectory() {
  auto rt = std::make_shared<moveit_msgs::msg::RobotTrajectory>();
  trajectory_msgs::msg::JointTrajectoryPoint a, b, c, d;
  a.time_from_start.nanosec = 1e8;
  a.positions.push_back(1.0);
  a.velocities.push_back(0.0);
  b.time_from_start.nanosec = 2e8;
  b.positions.push_back(2.0);
  b.velocities.push_back(0.0);
  c.time_from_start.nanosec = 3e8;
  c.positions.push_back(3.0);
  c.velocities.push_back(0.0);
  d.time_from_start.nanosec = 4e8;
  d.positions.push_back(4.0);
  d.velocities.push_back(0.0);

  rt->joint_trajectory.points = {a, b, c, d};
  return rt;
}

void RegularTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  tt.UpdateTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void NonMonotonicTimeTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.joint_trajectory.points;
  std::swap(points[1], points[2]);
  tt.UpdateTrajectory(mock_rt);

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void OnePointTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  moveit_msgs::msg::RobotTrajectory mock_rt{};
  trajectory_msgs::msg::JointTrajectoryPoint a;
  a.time_from_start.sec = 0;
  a.positions.push_back(1.0);

  mock_rt.joint_trajectory.points = {a};

  tt.UpdateTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void TwoPointTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  moveit_msgs::msg::RobotTrajectory mock_rt{};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.time_from_start.sec = 0;
  a.positions.push_back(1.0);
  a.velocities.push_back(0.0);
  b.time_from_start.sec = 1;
  b.positions.push_back(2.0);
  b.velocities.push_back(0.0);

  mock_rt.joint_trajectory.points = {a, b};

  tt.UpdateTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void NegativeTimeTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.joint_trajectory.points;
  points[0].time_from_start.sec = -4;
  points[1].time_from_start.sec = -3;
  points[2].time_from_start.sec = -2;
  points[3].time_from_start.sec = -1;
  tt.UpdateTrajectory(mock_rt);

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void EmptyTrajectoryTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  moveit_msgs::msg::RobotTrajectory mock_rt{};

  tt.UpdateTrajectory(mock_rt);

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void JointNumberMismatchTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.joint_trajectory.points;
  points[0].positions.push_back(1.0);
  tt.UpdateTrajectory(mock_rt);

  const auto start_time = std::chrono::steady_clock::now();
  for (;;) {
    tt.Update();
    std::cout << tt.current_point().positions[0] << std::endl;
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

int main() {
  info("TEST 1: Regular test");
  RegularTest();
  info("TEST 2: Non-monotonic time");
  NonMonotonicTimeTest();
  info("TEST 3: One point");
  OnePointTest();
  info("TEST 4: Two points");
  TwoPointTest();
  info("TEST 5: Negative time");
  NegativeTimeTest();
  info("TEST 6: Empty trajectory");
  EmptyTrajectoryTest();
  info("TEST 7: Joint number mismatch");
  JointNumberMismatchTest();
  return 0;
}