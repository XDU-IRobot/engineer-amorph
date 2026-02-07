
#pragma once

#include <array>

#include <librm.hpp>

#include "dynamics.hpp"
#include "trajectory_tracker.hpp"

template <size_t NJoints>
class ArmController {
 public:
  struct Output {
    static Output Zero() {
      Output out;
      out.positions.fill(0.f);
      out.velocities.fill(0.f);
      out.efforts.fill(0.f);
      return out;
    }
    std::array<float, NJoints> positions;   ///< 关节位置输出，单位：rad
    std::array<float, NJoints> velocities;  ///< 关节速度输出，单位：rad/s
    std::array<float, NJoints> efforts;     ///< 关节力矩输出，单位：Nm
  };

  enum Status {
    kDisabled,
    kIdle,
    kExecuting,
    kFault,
  };

  ArmController() {
    // clang-format off
    pid_[0].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    pid_[1].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    pid_[2].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    pid_[3].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    pid_[4].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    pid_[5].SetKp(2.f).SetKi(0.f).SetKd(0.f).SetMaxOut(1000.f).SetDiffFirst(true);
    pid_[6].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f);
    // clang-format on
  }

  void Execute(const JointTrajectory &traj) {
    trajectory_tracker_.SwitchReferenceTrajectory(traj);
    // for (size_t i = 0; i < NJoints; ++i) {
    //   speed_pid_[i].Clear();
    //   position_pid_[i].Clear();
    // }
    UpdateStatus();
  }

  void GoTo(const JointTrajectoryPoint &point) {
    trajectory_tracker_.SwitchReferenceTrajectory(point);
    // for (size_t i = 0; i < NJoints; ++i) {
    //   speed_pid_[i].Clear();
    //   position_pid_[i].Clear();
    // }
    UpdateStatus();
  }

  void Update(const std::array<float, NJoints> &current_positions, const std::array<float, NJoints> &current_velocities,
              float dt = 1.f) {
    current_positions_ = current_positions;
    trajectory_tracker_.Update();
    const auto &ref_point = trajectory_tracker_.current_point();

    for (size_t i = 0; i < NJoints; ++i) {
      // 应用轨迹限制器，生成平滑的参考位置和速度
      trajectory_limiter_[i].SetTarget(ref_point.positions[i]);
      trajectory_limiter_[i].Update(dt);
      // // 位置环PID
      // position_pid_[i].Update(trajectory_limiter_[i].Update(dt), current_positions[i], dt);
      // // 速度环PID
      // speed_pid_[i].Update(trajectory_limiter_[i].current_velocity() + position_pid_[i].out(), current_velocities[i],
      //                      dt);
      // // 输出
      output_.positions[i] = trajectory_limiter_[i].current_position();
      // output_.velocities[i] = trajectory_limiter_[i].current_velocity();
      // output_.efforts[i] = speed_pid_[i].out();
      output_.velocities[i] = 0.f;
      output_.efforts[i] = 0.f;
    }

    // 对j2 j3叠加动力学前馈力矩
    const auto feedforward_efforts =
        dynamic_model_.Inverse({current_positions[0], current_positions[1], current_positions[2]});
    for (size_t i = 1; i < 3; ++i) {
      output_.efforts[i] += feedforward_efforts(i);
    }
    // output_.efforts[1] = -output_.efforts[1];  // j2反向安装
    UpdateStatus();

    if (status_ == kFault || status_ == kDisabled) {  // 故障状态和控制器禁用状态下输出全零
      // output_.positions = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // 位置不要动，保持输出故障之前的原位置
      output_.velocities.fill(0.f);
      output_.efforts.fill(0.f);
    }
  }

  void ResetAt(const std::array<float, NJoints> &positions) {
    current_positions_ = positions;
    trajectory_tracker_.SwitchReferenceTrajectory([&] {
      JointTrajectoryPoint point;
      point.positions = std::vector(positions.begin(), positions.end());
      point.velocities.resize(NJoints, 0.f);
      return point;
    }());
    for (size_t i = 0; i < NJoints; ++i) {
      trajectory_limiter_[i].ResetAt(positions[i]);
      pid_[i].Clear();
    }
  }

  /**
   * @brief 控制器进入故障状态之后，如果确认故障已经排除，可以调用此函数清除故障状态
   * @note  把这个函数绑定到遥控器的一个通道上，当机械臂进入故障状态后，可以且仅能通过遥控器来清除故障状态
   */
  void FaultClear() {
    if (status_ == kFault) {
      ResetAt(current_positions_);
      fault_counter_ = 0;
      status_ = kDisabled;  // 安全起见，先fault -> disabled，需要再手动操作一次进入idle状态
    }
  }

  /**
   * @brief 启用或禁用控制器
   * @param enable 传入 true 启用控制器，传入 false 禁用控制器
   */
  void Enable(bool enable) {
    if (status_ == kFault) {
      return;  // 故障状态下不允许启用或禁用控制器
    }
    if (!enable) {
      status_ = kDisabled;
      return;
    }
    if (status_ == kDisabled) {
      ResetAt(current_positions_);
      status_ = kIdle;
    }
  }

  static const auto &joint_limits() { return kJointLimits; }
  const auto &current_positions() const { return current_positions_; }
  const auto &output() const { return output_; }
  Status status() const { return status_; }
  auto &pid() { return pid_; }

 private:
  void UpdateStatus() {
    if (status_ == kDisabled) {
      return;  // 控制器禁用状态下不更新状态
    }

    const bool all_limiters_converged = std::ranges::all_of(
        trajectory_limiter_, [](const rm::modules::TrajectoryLimiter &limiter) { return limiter.IsAtTarget(); });
    const bool tracker_finished = trajectory_tracker_.IsTrajectoryFinished();
    if (tracker_finished && all_limiters_converged) {
      // 方便调试先禁用掉下面检测故障的逻辑
      status_ = kIdle;
      return;

      const auto position_delta = [&] {
        const auto target_position = trajectory_tracker_.current_point().positions;
        std::array<float, NJoints> deltas;
        for (size_t i = 0; i < NJoints; ++i) {
          deltas[i] = std::abs(target_position[i] - current_positions_[i]);
        }
        return deltas;
      }();
      const bool arm_is_actually_at_target =
          std::ranges::all_of(position_delta, [](const float delta) { return delta < 0.1f; });  // 0.1 rad 容差
      if (arm_is_actually_at_target) {
        // tracker和limiter都已经执行到位，机械臂也确实到达了目标位置，认为任务完成，进入空闲状态
        fault_counter_ = 0;
        status_ = kIdle;
        return;
      }
      fault_counter_++;  // tracker和limiter都已经执行到位，但机械臂并没有到达目标位置，递增一次故障计数
      status_ =
          fault_counter_ >= 500
              ? kFault  // 500个周期仍然没有到达目标，可能是撞到东西了，也可能是电机离线没有反馈了，总之进入故障状态停止运行
              : kExecuting;  // 500个周期以内暂时认为机械臂还在执行中
      return;
    }
    status_ = kExecuting;
  }

  std::array<float, NJoints> current_positions_{};
  Output output_{};
  TrajectoryTracker<NJoints> trajectory_tracker_{kJointLimits[0], kJointLimits[1], kJointLimits[2], kJointLimits[3],
                                                 kJointLimits[4], kJointLimits[5], kJointLimits[6]};
  DynamicModel dynamic_model_;
  std::array<rm::modules::TrajectoryLimiter, NJoints> trajectory_limiter_  //
      {rm::modules::TrajectoryLimiter{2.175f, 3.75f},                      //
       rm::modules::TrajectoryLimiter{2.175f, 1.875f},                     //
       rm::modules::TrajectoryLimiter{2.175f, 2.5f},                       //
       rm::modules::TrajectoryLimiter{2.175f, 3.125f},                     //
       rm::modules::TrajectoryLimiter{2.61f, 3.75f},                       //
       rm::modules::TrajectoryLimiter{2.61f, 5.f},                         //
       rm::modules::TrajectoryLimiter{2.61f, 5.f}};
  std::array<rm::modules::PID, NJoints> pid_;

  constexpr static std::array<std::pair<float, float>, NJoints> kJointLimits{
      std::pair{-M_PI, M_PI}, std::pair{-M_PI, M_PI}, std::pair{-M_PI, M_PI}, std::pair{-M_PI, M_PI},
      std::pair{-M_PI, M_PI}, std::pair{-M_PI, M_PI}, std::pair{-M_PI, M_PI}};  // placeholder joint limits

  Status status_{kIdle};
  int fault_counter_{0};
};