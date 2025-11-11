
#pragma once

#include <librm.hpp>
#include <etl/array.h>

#include "dynamics.hpp"
#include "trajectory_tracker.hpp"

class ArmController {
 public:
  template <size_t NJoints>
  struct Output {
    etl::array<float, NJoints> velocities;  ///< 关节速度输出，单位：rad/s
    etl::array<float, NJoints> efforts;     ///< 关节力矩输出，单位：Nm
  };

  enum Status {
    kIdle,
    kExecuting,
    kFault,
  };

 public:
  ArmController() {
    // clang-format off
    position_pid_[0].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    position_pid_[1].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    position_pid_[2].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    position_pid_[3].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    position_pid_[4].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    position_pid_[5].SetKp(20.f).SetKi(0.f).SetKd(0.5f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[0].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[1].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[2].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[3].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[4].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    speed_pid_[5].SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.f).SetDiffFirst(true).SetFuzzy(true).SetFuzzyErrorScale(M_PI);
    // clang-format on
  }

  void Execute(const JointTrajectory &traj) { trajectory_tracker_.SwitchReferenceTrajectory(traj); }

  void GoTo(const JointTrajectoryPoint &point) { trajectory_tracker_.SwitchReferenceTrajectory(point); }

  void Update(etl::array<float, 6> current_positions, etl::array<float, 6> current_velocities, float dt = 1.f) {
    current_positions_ = current_positions;
    trajectory_tracker_.Update();
    const auto &ref_point = trajectory_tracker_.current_point();

    for (size_t i = 0; i < 6; ++i) {
      // 轨迹限幅
      trajectory_limiter_[i].SetTarget(ref_point.positions[i]);
      // 位置环PID
      position_pid_[i].Update(trajectory_limiter_[i].Update(current_positions[i]), current_positions[i], dt);
      // 速度环PID
      speed_pid_[i].Update(ref_point.velocities[i] + position_pid_[i].out(), current_velocities[i], dt);
      // 输出
      output_.velocities[i] = ref_point.velocities[i];
      output_.efforts[i] = speed_pid_[i].out();
    }

    // 对前三个关节叠加动力学前馈力矩
    const auto feedforward_efforts =
        dynamic_model_.Inverse({current_positions[0], current_positions[1], current_positions[2]});
    for (size_t i = 0; i < 3; ++i) {
      output_.efforts[i] += feedforward_efforts[i];
    }
  }

  const auto &output() const { return output_; }
  Status status() const {
    const bool all_limiters_converged = std::ranges::all_of(
        trajectory_limiter_, [](const rm::modules::TrajectoryLimiter &limiter) { return !limiter.IsAtTarget(); });
    const bool tracker_finished = trajectory_tracker_.IsTrajectoryFinished();

    if (tracker_finished || all_limiters_converged) {
      const auto position_delta = [&] {
        const auto target_position = trajectory_tracker_.current_point().positions;
        etl::array<float, 6> deltas;
        for (size_t i = 0; i < 6; ++i) {
          deltas[i] = std::abs(target_position[i] - current_positions_[i]);
        }
        return deltas;
      }();
      const bool arm_is_actually_at_target =
          std::ranges::all_of(position_delta, [](const float delta) { return delta < 0.1f; });  // 0.1 rad 容差
      if (arm_is_actually_at_target) {
        return kIdle;
      }
      return kFault;  // 很有可能是撞到东西卡住了
    }
    return kExecuting;
  }

 private:
  etl::array<float, 6> current_positions_;
  Output<6> output_{};
  TrajectoryTracker<6> trajectory_tracker_{};
  DynamicModel dynamic_model_;
  etl::array<rm::modules::TrajectoryLimiter, 6> trajectory_limiter_  //
      {rm::modules::TrajectoryLimiter{1.f, 1.f},                     //
       rm::modules::TrajectoryLimiter{1.f, 1.f},                     //
       rm::modules::TrajectoryLimiter{1.f, 1.f},                     //
       rm::modules::TrajectoryLimiter{1.f, 1.f},                     //
       rm::modules::TrajectoryLimiter{1.f, 1.f},                     //
       rm::modules::TrajectoryLimiter{1.f, 1.f}};
  etl::array<rm::modules::PID, 6> speed_pid_;
  etl::array<rm::modules::PID, 6> position_pid_;
};