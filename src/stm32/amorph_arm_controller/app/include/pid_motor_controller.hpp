
#pragma once

#include <librm.hpp>

#include <utility>

using namespace rm::modules::angle_literals;

/**
 * @brief   基于PID控制的电机位置控制器
 * @tparam  MotorType 底层电机驱动类类型
 */
template <typename MotorType>
class PIDMotorController {
 public:
  /**
   * @brief             构造函数，接受任意数量和类型的参数，并将其完美转发给底层电机驱动类的构造函数
   * @tparam    Args    可变参数模板，用于捕获任意数量和类型的参数
   * @param     args    参数包，将被完美转发给底层电机控制器
   */
  template <typename... Args>
  explicit PIDMotorController(Args &&...args) : motor_(std::forward<Args>(args)...) {}

  auto &motor() { return motor_; }
  auto &speed_pid() { return speed_pid_; }
  auto &position_pid() { return position_pid_; }

  /**
   * @brief 启用或禁用速度环PID控制器
   * @param enable
   * 如果为true，则启用速度环PID控制器，实现双环控制；如果为false，则禁用速度环PID控制器，仅使用位置环PID控制器，实现单环控制
   */
  void EnableSpeedPID(bool enable) { enable_speed_pid_ = enable; }

  /**
   * @brief 设置目标位置
   * @param position 期望位置
   */
  void MoveTo(rm::modules::Angle position) { desired_position_ = position; }

  /**
   * @brief 更新控制器状态并发送控制命令
   * @param dt 时间步长，默认为1
   */
  void Update(float dt = 1.f) {
    if (enable_speed_pid_) {
      // 双环控制
      position_pid_.Update(desired_position_.rad(), GetCurrentPosition().rad(), dt);
      const float desired_speed = position_pid_.out();
      speed_pid_.Update(desired_speed, GetCurrentSpeed(), dt);
      const float torque_command = speed_pid_.out();
      SendTorqueCommand(torque_command);
    } else {
      // 单环控制
      position_pid_.Update(desired_position_.rad(), GetCurrentPosition().rad(), dt);
      const float torque_command = position_pid_.out();
      SendTorqueCommand(torque_command);
    }
  }

  /**
   * @brief 重置PID控制器的状态
   */
  void Reset() {
    speed_pid_.Clear();
    position_pid_.Clear();
  }

 private:
  // 下面几个函数根据不同的电机类型进行特化
  void SendTorqueCommand(float torque);
  rm::modules::Angle GetCurrentPosition();
  float GetCurrentSpeed();

 private:
  MotorType motor_;
  rm::modules::Angle desired_position_;
  rm::modules::PID speed_pid_, position_pid_;
  bool enable_speed_pid_{false};  ///< 双环or单环
};

// 特化不同电机类的SendTorqueCommand函数
template <>
inline void PIDMotorController<rm::device::GM6020>::SendTorqueCommand(float torque) {
  motor_.SetCurrent(torque);
}

template <>
inline void PIDMotorController<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>::SendTorqueCommand(
    float torque) {
  motor_.SetPosition(0, 0, torque, 0, 0);
}

// 特化不同电机类的GetCurrentPosition函数
template <>
inline rm::modules::Angle PIDMotorController<rm::device::GM6020>::GetCurrentPosition() {
  return rm::modules::Angle::FromDeg(motor_.pos_rad());
}

template <>
inline rm::modules::Angle
PIDMotorController<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>::GetCurrentPosition() {
  return rm::modules::Angle::FromRad(motor_.pos());
}

// 特化不同电机类的GetCurrentSpeed函数
template <>
inline float PIDMotorController<rm::device::GM6020>::GetCurrentSpeed() {
  return motor_.rpm();
}

template <>
inline float PIDMotorController<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>::GetCurrentSpeed() {
  return motor_.vel();
}