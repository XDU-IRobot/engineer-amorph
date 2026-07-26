#pragma once

#include <array>

#include <librm.hpp>

#include "cartesian_limit.hpp"
#include "dynamics.hpp"

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
    std::array<float, 6> positions;   ///< 关节位置输出，单位：rad
    std::array<float, 6> velocities;  ///< 关节速度输出，单位：rad/s
    std::array<float, 6> efforts;     ///< 关节力矩输出，单位：Nm
  };

  enum Status {
    kIdle,
    kMoving,
    kFault,
  };

  ArmController() = delete;
  explicit ArmController(std::array<std::pair<float, float>, 6> joint_limits,  //
                         std::array<rm::modules::TrajectoryLimiter, 6> trajectory_limiters,
                         ArmDynamics::Params dynamics_params)
      : trajectory_limiter_(std::move(trajectory_limiters)),
        joint_limits_(std::move(joint_limits)),
        dynamics_(dynamics_params) {
    // clang-format off
    pid_[0].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[1].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[2].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[3].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[4].SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(0.f).SetDiffFirst(true);
    pid_[5].SetKp(50000.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10000.f).SetDiffFirst(true);
    // clang-format on
    for (auto &f : acc_lpf_) {
      f.set_cutoff_frequency(500.f, 9.f);
    }
  }

  void GoTo(std::array<float, 6> target_positions) {
    const auto x_limited = cartesian_limiter_.Limit(target_positions[0], target_positions[1], target_positions[2]);
    if (!x_limited.has_value()) {
      return;  // 位置不可达
    }
    target_positions[1] = x_limited.value()[0];
    target_positions[2] = x_limited.value()[1];
    for (size_t i = 0; i < 6; ++i) {
      trajectory_limiter_[i].SetTarget(
          std::clamp(target_positions[i], joint_limits_[i].first, joint_limits_[i].second));
    }
    UpdateStatus();
  }

  void Update(const std::array<float, 6> &current_positions, const std::array<float, 6> &current_velocities,
              Eigen::Vector3f imu_acc, float dt = 1.f) {
    current_positions_ = current_positions;

    for (size_t i = 0; i < 6; ++i) {
      trajectory_limiter_[i].Update(dt);
      output_.positions[i] = trajectory_limiter_[i].current_position();
      output_.velocities[i] = trajectory_limiter_[i].current_velocity();
      output_.efforts[i] = 0.f;
    }

    pid_[5].UpdateExtDiff(output_.positions[5], current_positions[5], current_velocities[5], dt);
    output_.efforts[5] = pid_[5].out();

    const float acc_x = acc_lpf_[0].apply(imu_acc[0]);
    const float acc_y = acc_lpf_[1].apply(imu_acc[1]);
    const float acc_z = acc_lpf_[2].apply(imu_acc[2]);

    // 对j1 j2 j3叠加动力学前馈力矩
    const auto acc_comp_efforts =
        dynamics_.Compensate({current_positions[0], current_positions[1], current_positions[2]}, {acc_x, acc_y, acc_z});
    for (int i = 0; i < 3; ++i) {
      output_.efforts[i] += acc_comp_efforts(i);
      output_.efforts[i] = std::clamp(output_.efforts[i], -20.f, 20.f);  // 限幅
    }

    UpdateStatus();

    if (status_ == kFault) {  // 故障状态下输出全零
      output_.velocities.fill(0.f);
      output_.efforts.fill(0.f);
    }
  }

  void ResetAt(const std::array<float, 6> &positions) {
    current_positions_ = positions;
    for (size_t i = 0; i < 6; ++i) {
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

  const auto &joint_limits() { return joint_limits_; }
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
        std::array<float, 6> deltas;
        for (size_t i = 0; i < 6; ++i) {
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

  std::array<float, 6> current_positions_{};
  Output output_{};
  std::array<rm::modules::TrajectoryLimiter, 6> trajectory_limiter_;
  std::array<rm::modules::PID, 6> pid_;
  std::array<std::pair<float, float>, 6> joint_limits_;
  CartesianLimit cartesian_limiter_{0.479754f, 0.45f};
  ArmDynamics dynamics_;
  rm::modules::LowPassFilter2p<float> acc_lpf_[3];
  Status status_{kIdle};
  int fault_counter_{0};
};