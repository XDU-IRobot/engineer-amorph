
#include "arm/pid_trajectory_tracker.hpp"
#include "common/logger.hpp"

#include <chrono>
#include <iostream>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>

std::shared_ptr<trajectory_msgs::msg::JointTrajectory> GenerateValidTrajectory() {
  auto rt = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
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

  rt->points = {a, b, c, d};
  return rt;
}

template <int NJoints>
void RunTracker(PidTrajectoryTracker<NJoints> &tt) {
  using namespace std::chrono_literals;
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

void RegularTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  tt.SwitchReferenceTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void NonMonotonicTimeTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.points;
  std::swap(points[1], points[2]);
  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void OnePointTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  trajectory_msgs::msg::JointTrajectory mock_rt{};
  trajectory_msgs::msg::JointTrajectoryPoint a;
  a.time_from_start.sec = 0;
  a.positions.push_back(1.0);
  a.velocities.push_back(0.0);

  mock_rt.points = {a};

  tt.SwitchReferenceTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void TwoPointTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  trajectory_msgs::msg::JointTrajectory mock_rt{};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.time_from_start.sec = 0;
  a.positions.push_back(1.0);
  a.velocities.push_back(0.0);
  b.time_from_start.sec = 1;
  b.positions.push_back(2.0);
  b.velocities.push_back(0.0);

  mock_rt.points = {a, b};

  tt.SwitchReferenceTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void NegativeTimeTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.points;
  points[0].time_from_start.sec = -4;
  points[1].time_from_start.sec = -3;
  points[2].time_from_start.sec = -2;
  points[3].time_from_start.sec = -1;
  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void EmptyTrajectoryTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{PID{}, PID{}};
  trajectory_msgs::msg::JointTrajectory mock_rt{};

  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void JointNumberMismatchTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.points;
  points[0].positions.push_back(1.0);
  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void SinglePointJointNumberMismatchTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.time_from_start.sec = 0;
  p.positions.push_back(1.0);
  p.positions.push_back(2.0);
  p.positions.push_back(3.0);

  tt.SwitchReferenceTrajectory(p);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void PIDOnEmptyTrajectoryTest() {
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{PID{}, PID{}, PID{}, PID{}, PID{}, PID{}};

  auto mock_current_positions = std::array<double, 3>{1.0, 2.0, 3.0};
  auto mock_current_velocities = std::array<double, 3>{0.0, 0.0, 0.0};

  tt.Update(mock_current_positions, mock_current_velocities);

  RunTracker(tt);
}

void PIDNumberMismatchTest() {
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  try {
    PidTrajectoryTracker<3> tt{PID{}, PID{}, PID{}};
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
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
  info("TEST 8: Single point joint number mismatch");
  SinglePointJointNumberMismatchTest();
  info("TEST 9: PID on empty trajectory");
  PIDOnEmptyTrajectoryTest();
  info("TEST 10: PID number mismatch");
  PIDNumberMismatchTest();
  return 0;
}