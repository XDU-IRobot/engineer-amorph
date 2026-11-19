#pragma once

#include <array>

#include <Eigen/Dense>
#include <etl/vector.h>

#include "arm_controller.hpp"
#include "gripper_controller.hpp"
#include "kinematics.hpp"

/**
 * @brief 用链式调用构建一串动作，每个调用返回 *this 以便级联。通过ActionSequencer执行动作序列。
 * @code
 *   ActionSequence seq;
 *   seq.GoTo({0,0,0,0,0,0})
 *      .Delay(1000)
 *      .GripperOpen(true)
 *      .GoTo({1,1,1,1,1,1})
 *      .Delay(1000)
 *      .GripperOpen(false)
 *      .GoTo({0,0,0,0,0,0});
 * @endcode
 */
class ActionSequence {
 public:
  constexpr static size_t kMaxSteps = 20;

  enum StepType : uint8_t {
    kGoTo,                   ///< 关节空间轨迹运动
    kDelay,                  ///< 等待（ms）
    kGripperOpen,            ///< 夹爪开/合
    kCartesianRelativeGoTo,  ///< 笛卡尔空间相对直线运动
  };

  struct Step {
    StepType type;
    std::array<float, 6> positions{};  ///< kGoTo 目标关节角（rad）; kCartesianRelativeGoTo 相对位移（m）[dx,dy,dz]
    bool gripper_open{};               ///< kGripperOpen 目标状态
    uint32_t delay_ms{};               ///< kDelay 时长（ms）
  };

  ActionSequence() = default;

  ActionSequence &GoTo(const std::array<float, 6> &target) {
    if (steps_.full()) return *this;
    steps_.push_back({kGoTo, target, false, 0});
    return *this;
  }

  ActionSequence &Delay(uint32_t ms) {
    if (steps_.full()) return *this;
    steps_.push_back({kDelay, {}, false, ms});
    return *this;
  }

  ActionSequence &GripperOpen(bool open) {
    if (steps_.full()) return *this;
    steps_.push_back({kGripperOpen, {}, open, 0});
    return *this;
  }

  ActionSequence &CartesianRelativeGoTo(float dx, float dy, float dz) {
    if (steps_.full()) return *this;
    Step s;
    s.type = kCartesianRelativeGoTo;
    s.positions[0] = dx;
    s.positions[1] = dy;
    s.positions[2] = dz;
    steps_.push_back(s);
    return *this;
  }

  const auto &steps() const { return steps_; }
  void Clear() { steps_.clear(); }

 private:
  etl::vector<Step, kMaxSteps> steps_;
};

/**
 * @brief 动作序列执行器
 *
 * FSM Sequencing 状态持有此对象，通过引用操作 ArmController 和
 * GripperController。每个 ControlLoop 中调用一次 Tick()，并通过
 * IsFinished() 判断序列是否结束。
 */
class ActionSequencer {
 public:
  ActionSequencer(ArmController &arm, GripperController<> &gripper) : arm_(&arm), gripper_(&gripper) {}

  /// 加载并立即开始执行
  void Execute(const ActionSequence &seq) {
    steps_ = seq.steps();
    current_step_ = 0;
    step_elapsed_ms_ = 0;
    step_started_ = false;
    finished_ = steps_.empty();
  }

  /**
   * @brief 每个控制周期调用一次
   * @param dt_ms 控制周期（毫秒），用于 kDelay 计时
   */
  void Update(uint32_t dt_ms) {
    if (finished_) return;

    if (!step_started_) {
      // 启动当前步 — 仅下发命令，不调用 Update
      const auto &s = steps_[current_step_];
      switch (s.type) {
        case ActionSequence::kGoTo:
          arm_->GoTo(s.positions);
          break;
        case ActionSequence::kDelay:
          break;
        case ActionSequence::kGripperOpen:
          gripper_->Open(s.gripper_open);
          break;
        case ActionSequence::kCartesianRelativeGoTo: {
          // 通过正运动学获取当前末端位姿，计算目标位姿，初始化笛卡尔插补器
          const auto joint_arr = arm_->current_positions();
          Eigen::Matrix<float, 6, 1> joints;
          for (int i = 0; i < 6; ++i) joints(i) = joint_arr[i];

          const Eigen::Matrix4f T_current = kinematics_.SolveFk(joints);
          const Eigen::Vector3f current_pos = T_current.block<3, 1>(0, 3);
          cartesian_target_rot_ = T_current.block<3, 3>(0, 0);
          cartesian_target_ = current_pos + Eigen::Vector3f(s.positions[0], s.positions[1], s.positions[2]);

          for (int i = 0; i < 3; ++i) {
            cartesian_interp_[i].ResetAt(current_pos[i]);
            cartesian_interp_[i].SetTarget(cartesian_target_[i]);
          }
          break;
        }
      }
      step_started_ = true;
      step_elapsed_ms_ = 0;
    }

    // 对于需要每个周期连续更新指令的步骤类型，在此执行插补
    {
      const auto &s = steps_[current_step_];
      if (s.type == ActionSequence::kCartesianRelativeGoTo) {
        const float dt_s = static_cast<float>(dt_ms) * 0.001f;
        Eigen::Vector3f interp_pos;
        for (int i = 0; i < 3; ++i) {
          interp_pos[i] = cartesian_interp_[i].Update(dt_s);
        }
        // 逆运动学求解：保持当前姿态，仅平移插补位置
        Eigen::Matrix4f T_target = Eigen::Matrix4f::Identity();
        T_target.block<3, 3>(0, 0) = cartesian_target_rot_;
        T_target.block<3, 1>(0, 3) = interp_pos;

        const auto joint_arr = arm_->current_positions();
        Eigen::Matrix<float, 6, 1> current_joints;
        for (int i = 0; i < 6; ++i) current_joints(i) = joint_arr[i];

        auto ik_result = kinematics_.SolveIk(T_target, current_joints);
        if (ik_result.has_value()) {
          std::array<float, 6> target_joints;
          for (int i = 0; i < 6; ++i) target_joints[i] = ik_result.value()(i);
          arm_->GoTo(target_joints);
        }
      }
    }

    // 检查当前步是否完成
    bool step_complete = false;
    const auto &s = steps_[current_step_];
    switch (s.type) {
      case ActionSequence::kGoTo:
        // 由 FSM 层调用 ArmController::Update 后，status 不再是 kMoving
        step_complete = (arm_->status() != ArmController::kMoving);
        break;
      case ActionSequence::kDelay:
        step_complete = (step_elapsed_ms_ >= s.delay_ms);
        break;
      case ActionSequence::kGripperOpen:
        // 夹爪正在执行动作（含校准）时不算完成
        step_complete = !gripper_->IsBusy();
        break;
      case ActionSequence::kCartesianRelativeGoTo: {
        // 所有轴插补到位 且 机械臂不再运动中
        bool all_at_target = true;
        for (int i = 0; i < 3; ++i) {
          if (!cartesian_interp_[i].IsAtTarget(0.001f)) {
            all_at_target = false;
            break;
          }
        }
        step_complete = all_at_target && (arm_->status() != ArmController::kMoving);
        break;
      }
    }

    if (step_complete) {
      current_step_++;
      step_started_ = false;
      if (current_step_ >= steps_.size()) {
        finished_ = true;
      }
    }

    step_elapsed_ms_ += dt_ms;
  }

  bool IsFinished() const { return finished_; }

 private:
  ArmController *arm_;
  GripperController<> *gripper_;
  etl::vector<ActionSequence::Step, ActionSequence::kMaxSteps> steps_;
  size_t current_step_{0};
  uint32_t step_elapsed_ms_{0};
  bool step_started_{false};
  bool finished_{true};

  /// 笛卡尔空间轨迹插补器（X/Y/Z 轴各一个）
  ArmKinematics kinematics_;
  std::array<rm::modules::TrajectoryLimiter, 3> cartesian_interp_{{
      {0.5f, 1.f},
      {0.5f, 1.f},
      {0.5f, 1.f},
  }};
  Eigen::Vector3f cartesian_target_{0.f, 0.f, 0.f};
  Eigen::Matrix3f cartesian_target_rot_{Eigen::Matrix3f::Identity()};
};
