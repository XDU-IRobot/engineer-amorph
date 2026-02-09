
#pragma once

#include <array>

#include <librm.hpp>

#include "dynamics.hpp"

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
    kIdle,
    kMoving,
    kFault,
  };

  ArmController() {
    // clang-format off
    pid_[0].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[1].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[2].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[3].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[4].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[5].SetKp(2.f).SetKi(0.f).SetKd(0.f).SetMaxOut(1000.f).SetDiffFirst(true);
    pid_[6].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    // clang-format on
  }

  void GoTo(const std::array<float, NJoints> &target_positions) {
    for (size_t i = 0; i < NJoints; ++i) {
      trajectory_limiter_[i].SetTarget(std::clamp(target_positions[i], kJointLimits[i].first, kJointLimits[i].second));
    }
    UpdateStatus();
  }

  void Update(const std::array<float, NJoints> &current_positions, const std::array<float, NJoints> &current_velocities,
              float dt = 1.f) {
    current_positions_ = current_positions;

    for (size_t i = 0; i < NJoints; ++i) {
      // 应用轨迹限制器，生成平滑的参考位置和速度
      trajectory_limiter_[i].Update(dt);

      // 输出当前 limiter 的位置和速度
      output_.positions[i] = trajectory_limiter_[i].current_position();
      output_.velocities[i] = 0.f;
      output_.efforts[i] = 0.f;
    }

    // 对j2 j3叠加动力学前馈力矩
    const auto feedforward_efforts =
        dynamic_model_.Inverse({current_positions[0], current_positions[1], current_positions[2]});
    for (size_t i = 1; i < 3; ++i) {
      output_.efforts[i] += feedforward_efforts(i);
    }

    UpdateStatus();

    if (status_ == kFault) {  // 故障状态下输出全零
      output_.velocities.fill(0.f);
      output_.efforts.fill(0.f);
    }
  }

  void ResetAt(const std::array<float, NJoints> &positions) {
    current_positions_ = positions;
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
    const bool all_limiters_converged = std::ranges::all_of(
        trajectory_limiter_, [](const rm::modules::TrajectoryLimiter &limiter) { return limiter.IsAtTarget(); });

    if (all_limiters_converged) {
      // 方便调试先禁用下面的判断逻辑
      status_ = kIdle;
      return;

      // 所有 limiter 都已经到达目标，检查机械臂是否真的到位
      const auto position_delta = [&] {
        std::array<float, NJoints> deltas;
        for (size_t i = 0; i < NJoints; ++i) {
          deltas[i] = std::abs(trajectory_limiter_[i].target_position() - current_positions_[i]);
        }
        return deltas;
      }();
      const bool arm_is_actually_at_target =
          std::ranges::all_of(position_delta, [](const float delta) { return delta < 0.1f; });  // 0.1 rad 容差
      if (arm_is_actually_at_target) {
        // limiter 已经执行到位，机械臂也确实到达了目标位置，认为任务完成，进入空闲状态
        fault_counter_ = 0;
        status_ = kIdle;
        return;
      }
      fault_counter_++;  // limiter 已经执行到位，但机械臂并没有到达目标位置，递增一次故障计数
      status_ =
          fault_counter_ >= 500
              ? kFault  // 500个周期仍然没有到达目标，可能是撞到东西了，也可能是电机离线没有反馈了，总之进入故障状态停止运行
              : kMoving;  // 500个周期以内暂时认为机械臂还在执行中
      return;
    }
    status_ = kMoving;
  }

  std::array<float, NJoints> current_positions_{};
  Output output_{};
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