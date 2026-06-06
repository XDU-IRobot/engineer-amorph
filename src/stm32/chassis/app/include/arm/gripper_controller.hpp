#pragma once

#include <type_traits>
#include <functional>

#include <etl/delegate.h>

#include "stm32h7xx_hal.h"

template <bool UseStdFunction = true>
class GripperController {
  // constexpr static float kMoveSpeed = 2.f * M_PI * 1.f;   ///< 夹爪开闭时驱动电机的速度，单位 rad/s
  // constexpr static float kCalibSpeed = 2.f * M_PI * 3.f;  ///< 校准时驱动电机的速度，单位 rad/s
  constexpr static float kCloseMoveSpeed = 2.f * M_PIf * 100.f;  ///< 夹爪开闭时驱动电机的速度，单位 rad/s
  constexpr static float kOpenMoveSpeed = 2.f * M_PIf * 100.f;   ///< 夹爪开闭时驱动电机的速度，单位 rad/s
  constexpr static float kCalibSpeed = 2.f * M_PIf * 30.f;       ///< 校准时驱动电机的速度，单位 rad/s
  constexpr static float kClosedEncoder = 0.f;                   ///< 夹爪完全闭合时编码器的读数
  constexpr static float kOpenEncoder = -110.f;                  ///< 夹爪完全打开时编码器的读数
  constexpr static uint32_t kCalibTimeoutMs = 2000;              ///< 校准超时时间，单位 ms
  constexpr static int kStallThresholdLoops = 20;                ///< 连续堵转多少次认为校准完成

 public:
  using CallbackType = std::conditional_t<UseStdFunction, std::function<void()>, etl::delegate<void()>>;

  enum State : uint8_t {
    kClose = 0,
    kOpen,
    kClosing,
    kOpening,
    kCalibrating,  ///< 正在校准编码器零点
  };

  GripperController() = default;

  /**
   * @brief 请求夹爪打开/关闭
   * @note  若尚未校准，会自动先进入校准流程，校准完成后执行被延迟的开关动作。
   *        外部调用者无需关心夹爪是否已校准。
   */
  void Open(bool open) {
    // 尚未校准 → 进入校准流程，缓存目标开关状态
    if (needs_calibration_) {
      if (state_ == kCalibrating) {
        // 校准进行中，只更新缓存目标
        pending_open_ = open;
        return;
      }
      pending_open_ = open;
      state_ = kCalibrating;
      calib_start_timestamp_ = HAL_GetTick();
      stall_count_ = 0;
      return;
    }

    if (state_ == kCalibrating) {
      // 校准进行中，只更新缓存目标
      pending_open_ = open;
      return;
    }

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

    // 校准状态处理 — 在 Update 中驱动校准流程，与正常的开闭动作共享同一条
    // actuator.gripper.SetSpeed() 输出通道（FSM 各状态不需要感知校准）。
    if (state_ == kCalibrating) {
      return UpdateCalibration();
    }

    moved_duration_ = (move_start_timestamp_ == 0) ? 0 : (HAL_GetTick() - move_start_timestamp_);

    const bool motor_stalled = CheckIsMotorStalled();
    if (state_ == kClose || state_ == kOpen) {
      target_speed_ = 0.f;
      return target_speed_;
    }

    if (state_ == kClosing) {
      target_speed_ = kCloseMoveSpeed;
      if (motor_stalled || motor_status_.position > kClosedEncoder) {
        state_ = kClose;
        target_speed_ = 0.f;
        if (on_action_finish_) {
          on_action_finish_();
        }
      }
    } else if (state_ == kOpening) {
      target_speed_ = -kOpenMoveSpeed;
      if (motor_stalled || motor_status_.position < kOpenEncoder) {
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

  /** @brief 注册校准完成回调（由 Arm::Init 注入硬件操作） */
  void OnCalibrationComplete(CallbackType cb) { on_calibration_complete_ = std::move(cb); }

  /** @brief 标记夹爪需要重新校准（设备离线时调用） */
  void SetNeedsCalibration(bool needs) { needs_calibration_ = needs; }

  bool motor_stalled() const { return motor_stalled_; }
  float output() const { return target_speed_; }
  State state() const { return state_; }
  bool needs_calibration() const { return needs_calibration_; }

  /** @brief 夹爪是否正在执行动作（开、合、校准中），供 ActionSequencer 判断步骤完成 */
  bool IsBusy() const { return state_ == kClosing || state_ == kOpening || state_ == kCalibrating; }

 protected:
  /**
   * @brief 校准流程 — 驱动电机正向转动直到堵转，更新编码器零点
   * @return 目标速度（由 FSM 通过 actuator.gripper.SetSpeed() 施加）
   */
  float UpdateCalibration() {
    const uint32_t calib_elapsed = HAL_GetTick() - calib_start_timestamp_;

    if (calib_elapsed > kCalibTimeoutMs) {
      // 超时：校准失败，回到闭合状态，下次 Open() 时重试
      target_speed_ = 0.f;
      state_ = kClose;
      needs_calibration_ = true;  // 保持校准需求标记
      return target_speed_;
    }

    if (std::abs(motor_status_.speed) < 25.f &&   // 转速低
        std::abs(motor_status_.power) > 690.f &&  // 力矩大
        calib_elapsed > 250) {                    // 过了初始加速阶段
      ++stall_count_;
    } else {
      stall_count_ = 0;
    }

    if (stall_count_ > kStallThresholdLoops) {
      // 校准完成：通知外部更新硬件零点
      if (on_calibration_complete_) {
        on_calibration_complete_();
      }
      needs_calibration_ = false;
      target_speed_ = 0.f;
      stall_count_ = 0;
      // 先退出校准状态，再调用 Open()，否则 Open() 内部的
      // state_ == kCalibrating 守卫会直接返回，导致 state_ 不更新
      // → Update() 再次进入 UpdateCalibration() → 无限递归爆栈
      state_ = kClose;
      Open(pending_open_);
      return Update(motor_status_.position, motor_status_.speed, motor_status_.power);
    }

    target_speed_ = kCalibSpeed;
    return target_speed_;
  }

  bool CheckIsMotorStalled() {
    if (std::abs(motor_status_.speed) < 25.f &&   // 转速低
        std::abs(motor_status_.power) > 690.f &&  // 力矩大
        moved_duration_ > 250)                    // 过了初始加速的阶段
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
  uint32_t last_no_stall_timestamp_{0u};  ///< 上次不堵转的时间戳
  uint32_t move_start_timestamp_{0};      ///< 夹爪开始移动的时间戳，单位ms
  uint32_t moved_duration_{0};            ///< 夹爪持续移动的时间，单位ms
  float target_speed_{0.f};
  State state_{kClose};
  CallbackType on_action_finish_{nullptr};
  CallbackType on_calibration_complete_{nullptr};

  // 校准相关成员
  bool needs_calibration_{true};       ///< 是否需要执行校准
  bool pending_open_{false};           ///< 校准期间缓存的 Open/Close 目标
  uint32_t calib_start_timestamp_{0};  ///< 校准开始时间戳，单位ms
  int stall_count_{0};                 ///< 校准过程中连续堵转计数
};