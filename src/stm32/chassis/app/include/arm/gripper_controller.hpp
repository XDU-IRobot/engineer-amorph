#pragma once

#include <type_traits>
#include <functional>

#include <etl/delegate.h>

#include "stm32h7xx_hal.h"

template <bool UseStdFunction = true>
class GripperController {
  constexpr static float kMoveSpeed = 2.f * M_PI * 100.f;  ///< 夹爪开闭时驱动电机的速度，单位 rad/s
  constexpr static float kClosedEncoder = 0.f;             ///< 夹爪完全闭合时编码器的读数
  constexpr static float kOpenEncoder = -110.f;             ///< 夹爪完全打开时编码器的读数

 public:
  using CallbackType = std::conditional_t<UseStdFunction, std::function<void()>, etl::delegate<void()>>;

  enum State : uint8_t {
    kClose = 0,
    kOpen,
    kClosing,
    kOpening,
  };

  GripperController() = default;

  void Open(bool open) {
    if (open) {
      switch (state_) {
        case kClose:
          move_start_timestamp_ = HAL_GetTick();
          state_ = kOpening;
          break;
        case kOpen:  // 已经是打开状态，不需要操作
          break;
        case kClosing:  // 正在关闭，改为打开
          move_start_timestamp_ = HAL_GetTick();
          state_ = kOpening;
          break;
        case kOpening:  // 已经在打开了，不需要操作
          break;
      }
    } else {
      switch (state_) {
        case kClose:  // 已经是关闭状态，不需要操作
          break;
        case kOpen:
          move_start_timestamp_ = HAL_GetTick();
          state_ = kClosing;
          break;
        case kClosing:  // 已经在关闭了，不需要操作
          break;
        case kOpening:  // 正在打开，改为关闭
          move_start_timestamp_ = HAL_GetTick();
          state_ = kClosing;
          break;
      }
    }
  }

  /**
   * @param current_position  夹爪驱动电机的当前位置
   * @param current_speed     夹爪驱动电机的当前速度
   * @param current_power     夹爪驱动电机的当前输出功率(ms4015, 0~850)
   * @return 夹爪驱动电机的目标速度
   */
  float Update(float current_position,  //
               float current_speed,     //
               float current_power) {
    motor_status_.position = current_position;
    motor_status_.speed = current_speed;
    motor_status_.power = current_power;
    moved_duration_ = (move_start_timestamp_ == 0) ? 0 : (HAL_GetTick() - move_start_timestamp_);

    const bool motor_stalled = CheckIsMotorStalled();
    if (state_ == kClose || state_ == kOpen) {
      target_speed_ = 0.f;
      return target_speed_;
    }

    if (state_ == kClosing) {
      target_speed_ = kMoveSpeed;
      if (motor_stalled || motor_status_.position > kClosedEncoder) {
        // 如果正在关闭过程中检测到堵转，或者编码器位置已经超过了完全打开的位置（说明夹爪已经完全打开了），认为关闭动作完成
        state_ = kClose;
        target_speed_ = 0.f;
        if (on_action_finish_) {
          on_action_finish_();
        }
      }
    } else if (state_ == kOpening) {
      target_speed_ = -kMoveSpeed;
      if (motor_stalled || motor_status_.position < kOpenEncoder) {
        // 如果正在打开过程中检测到堵转，或者编码器位置已经超过了完全闭合的位置（说明夹爪已经完全闭合了），认为打开动作完成
        state_ = kOpen;
        target_speed_ = 0.f;
        if (on_action_finish_) {
          on_action_finish_();
        }
      }
    }

    return target_speed_;
  }

  void ResetState(State state) { state_ = state; }

  void OnActionFinish(CallbackType cb) { on_action_finish_ = std::move(cb); }

  bool motor_stalled() const { return motor_stalled_; }
  float output() const { return target_speed_; }
  State state() const { return state_; }

 protected:
  bool CheckIsMotorStalled() {
    if (motor_status_.speed < 25.f &&  // 转速低
        motor_status_.power > 690 &&   // 力矩大
        moved_duration_ > 250)         // 过了初始加速的阶段
    {
    } else {
      motor_stalled_ = false;
      last_no_stall_timestamp_ = HAL_GetTick();
    }
    if (HAL_GetTick() - last_no_stall_timestamp_ > 1000) {  // 持续一段时间满足堵转条件则认为电机堵转
      motor_stalled_ = true;
    }
    return motor_stalled_;
  }

  struct MotorStatus {
    float position;
    float speed;
    float power;
  } motor_status_{};
  bool motor_stalled_{false};             ///< 电机堵转
  uint32_t last_no_stall_timestamp_{0u};  ///<
  uint32_t move_start_timestamp_{0};      ///< 夹爪开始移动的时间戳，单位ms
  uint32_t moved_duration_{0};            ///< 夹爪持续移动的时间，单位ms
  float target_speed_{0.f};
  State state_{kClose};
  CallbackType on_action_finish_{nullptr};
};