
#pragma once

#include <librm.hpp>

/**
 * @brief 有些“长”关节需要靠撞限位+堵转检测来校准位置，这个类实现了一个简单的长关节校准器
 */
class LongJointCalibrator {
  using CallbackType = std::function<void()>;

 public:
  LongJointCalibrator() = default;

  float Update(float current_position, float current_speed, float dt = 1.f) {
    using namespace std::chrono_literals;
    if (!calibrating_) {
      return 0.f;
    }
    if (std::chrono::steady_clock::now() - calibrate_start_time_ < 500ms) {
      // 开始校准500ms内不进行堵转检测，避免把电机加速判断为堵转
    }
    speed_pid_.Update(target_speed_, current_speed, dt);
    return speed_pid_.out();
  }

  void StartCalibration(float target_speed) {
    calibrating_ = true;
    target_speed_ = target_speed;
    calibrate_start_time_ = std::chrono::steady_clock::now();
  }

  void OnCalibrationComplete(CallbackType cb) { on_calibration_complete_ = std::move(cb); }

  float calibrated_pos_offset() const { return calibrated_pos_offset_; }
  auto &speed_pid() { return speed_pid_; }

 private:
  float calibrated_pos_offset_{0.f};
  float target_speed_{0.f};                                                   ///< 撞限位的速度
  bool calibrating_{false};                                                   ///< 是否正在校准
  rm::modules::PID speed_pid_;                                                ///< 速度环PID
  CallbackType on_calibration_complete_{};                                    ///< 校准完成回调
  std::chrono::time_point<std::chrono::steady_clock> calibrate_start_time_;   ///< 本次校准开始时间
  std::chrono::time_point<std::chrono::steady_clock> motor_stall_start_time;  ///< 电机堵转开始时间
};