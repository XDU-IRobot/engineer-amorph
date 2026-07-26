#pragma once

#include <array>

#include <etl/vector.h>

#include "arm_controller.hpp"
#include "gripper_controller.hpp"

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
    kGoTo,         ///< 关节空间轨迹运动
    kDelay,        ///< 等待（ms）
    kGripperOpen,  ///< 夹爪开/合
  };

  struct Step {
    StepType type;
    std::array<float, 6> positions{};  ///< kGoTo 目标关节角（rad）
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
      }
      step_started_ = true;
      step_elapsed_ms_ = 0;
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
};