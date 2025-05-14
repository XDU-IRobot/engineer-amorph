
#include "arm/pid_trajectory_tracker.hpp"
#include "common/logger.hpp"
#include "common/debug_helpers.hpp"

#include <chrono>
#include <iostream>

using fluorite::PidTrajectoryTracker;

std::shared_ptr<trajectory_msgs::msg::JointTrajectory> GenerateValidTrajectory() {
  auto rt = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msgs::msg::JointTrajectoryPoint a, b, c, d;
  a.time_from_start.nanosec = 1e8;
  a.positions.push_back(1.1);
  a.velocities.push_back(0.0);
  b.time_from_start.nanosec = 3e8;
  b.positions.push_back(2.2);
  b.velocities.push_back(0.0);
  c.time_from_start.nanosec = 5e8;
  c.positions.push_back(3.3);
  c.velocities.push_back(0.0);
  d.time_from_start.nanosec = 9e8;
  d.positions.push_back(4.4);
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
    Print(tt.current_point());
    if (std::chrono::steady_clock::now() - start_time > 1s) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
}

void RegularTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
  auto mock_rt = *GenerateValidTrajectory();

  std::cout << "ORIGINAL TRAJECTORY" << std::endl;
  Print(mock_rt);
  std::cout << "====================" << std::endl;

  tt.SwitchReferenceTrajectory(mock_rt);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void NonMonotonicTimeTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.points;
  std::swap(points[1], points[2]);
  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void SixDofTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<6> tt{
      {std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(),
       std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(),
       std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>()},
      {std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0),
       std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0)}};

  auto mock_rt = [] {
    auto rt = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    trajectory_msgs::msg::JointTrajectoryPoint a, b, c, d;
    a.time_from_start.nanosec = 1e8;
    a.positions = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
    a.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    b.time_from_start.nanosec = 3e8;
    b.positions = {7.7, 8.8, 9.9, 10.10, 11.11, 12.12};
    b.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    c.time_from_start.nanosec = 5e8;
    c.positions = {13.13, 14.14, 15.15, 16.16, 17.17, 18.18};
    c.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    d.time_from_start.nanosec = 9e8;
    d.positions = {19.19, 20.20, 21.21, 22.22, 23.23, 24.24};
    d.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rt->points = {a, b, c, d};
    return rt;
  }();

  tt.SwitchReferenceTrajectory(*mock_rt);
  tt.ResetReferenceTrajectory();

  RunTracker(tt);
}

void OnePointTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
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

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
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

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
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

  PidTrajectoryTracker<1> tt{{std::make_shared<PID>(), std::make_shared<PID>()}, {std::make_pair(-10000.0, 10000.0)}};
  trajectory_msgs::msg::JointTrajectory mock_rt{};

  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void JointNumberMismatchTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{
      {std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(),
       std::make_shared<PID>(), std::make_shared<PID>()},
      {std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0)}

  };
  auto mock_rt = *GenerateValidTrajectory();

  auto &points = mock_rt.points;
  points[0].positions.push_back(1.0);
  tt.SwitchReferenceTrajectory(mock_rt);

  RunTracker(tt);
}

void SinglePointJointNumberMismatchTest() {
  using namespace std::chrono_literals;
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  PidTrajectoryTracker<3> tt{
      {std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>(),
       std::make_shared<PID>(), std::make_shared<PID>()},
      {std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0)}};
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

  std::initializer_list<std::shared_ptr<PID>> pids{std::make_shared<PID>(), std::make_shared<PID>(),
                                                   std::make_shared<PID>(), std::make_shared<PID>(),
                                                   std::make_shared<PID>(), std::make_shared<PID>()};

  PidTrajectoryTracker<3> tt{
      pids, {std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0)}};

  const auto mock_current_positions = std::array<double, 3>{1.0, 2.0, 3.0};
  const auto mock_current_velocities = std::array<double, 3>{0.0, 0.0, 0.0};

  tt.Update(mock_current_positions, mock_current_velocities);

  RunTracker(tt);
}

void PIDNumberMismatchTest() {
  using PID = rm::modules::algorithm::PID<rm::modules::algorithm::PIDType::kPosition>;

  try {
    PidTrajectoryTracker<3> tt{
        {std::make_shared<PID>(), std::make_shared<PID>(), std::make_shared<PID>()},
        {std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0), std::make_pair(-10000.0, 10000.0)}};
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
  }
}

int main() {
  fluorite::info("TEST 1: Regular test");
  RegularTest();
  fluorite::info("TEST 2: Non-monotonic time");
  NonMonotonicTimeTest();
  fluorite::info("TEST 2.5: 6dof");
  SixDofTest();
  fluorite::info("TEST 3: One point");
  OnePointTest();
  fluorite::info("TEST 4: Two points");
  TwoPointTest();
  fluorite::info("TEST 5: Negative time");
  NegativeTimeTest();
  fluorite::info("TEST 6: Empty trajectory");
  EmptyTrajectoryTest();
  fluorite::info("TEST 7: Joint number mismatch");
  JointNumberMismatchTest();
  fluorite::info("TEST 8: Single point joint number mismatch");
  SinglePointJointNumberMismatchTest();
  fluorite::info("TEST 9: PID on empty trajectory");
  PIDOnEmptyTrajectoryTest();
  fluorite::info("TEST 10: PID number mismatch");
  PIDNumberMismatchTest();
  return 0;
}