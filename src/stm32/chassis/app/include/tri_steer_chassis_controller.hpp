#pragma once

#include <librm.hpp>

/**
 * @brief 三舵轮底盘控制器
 */
class TriSteerChassisController {
 public:
  explicit TriSteerChassisController(float chassis_radius = 0.2f) : kinematics_(chassis_radius) {
    pid_.f_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
    pid_.l_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
    pid_.r_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float front_steer_position, float front_steer_speed, float lr_steer_position, float lr_steer_speed,
              float rr_steer_position, float rr_steer_speed, float front_wheel_speed, float lr_wheel_speed,
              float rr_wheel_speed, float dt = 1.f) {
    // 更新当前状态
    state_.front_steer_position = front_steer_position;
    state_.lr_steer_position = lr_steer_position;
    state_.rr_steer_position = rr_steer_position;
    state_.front_steer_speed = front_steer_speed;
    state_.lr_steer_speed = lr_steer_speed;
    state_.rr_steer_speed = rr_steer_speed;
    state_.front_wheel_speed = front_wheel_speed;
    state_.lr_wheel_speed = lr_wheel_speed;
    state_.rr_wheel_speed = rr_wheel_speed;

    if (!enabled_) {
      output_.front_wheel = 0.f;
      output_.lr_wheel = 0.f;
      output_.rr_wheel = 0.f;
      output_.front_steer = 0.f;
      output_.lr_steer = 0.f;
      output_.rr_steer = 0.f;
      return;
    }

    // 运动学解算
    const auto fk_result = kinematics_.Forward(target_.vx, target_.vy, target_.w, state_.front_steer_position,
                                               state_.lr_steer_position, state_.rr_steer_position);

    // 舵
    pid_.f_steer.UpdateExtDiff(fk_result.front_steer_position, state_.front_steer_position, -state_.front_steer_speed,
                               dt);
    output_.front_steer = pid_.f_steer.out();
    pid_.l_steer.UpdateExtDiff(fk_result.lr_steer_position, state_.lr_steer_position, -state_.lr_steer_speed, dt);
    output_.lr_steer = pid_.l_steer.out();
    pid_.r_steer.UpdateExtDiff(fk_result.rr_steer_position, state_.rr_steer_position, -state_.rr_steer_speed, dt);
    output_.rr_steer = pid_.r_steer.out();

    // 轮
    pid_.f_wheel.Update(fk_result.front_wheel_speed, state_.front_wheel_speed, dt);
    output_.front_wheel = pid_.f_wheel.out();
    pid_.l_wheel.Update(fk_result.lr_wheel_speed, state_.lr_wheel_speed, dt);
    output_.lr_wheel = pid_.l_wheel.out();
    pid_.r_wheel.Update(fk_result.rr_wheel_speed, state_.rr_wheel_speed, dt);
    output_.rr_wheel = pid_.r_wheel.out();
  }

  /**
   * @brief 设置目标线速度和角速度，单位分别为米每秒和弧度每秒
   */
  void SetTarget(float vx, float vy, float w) {
    target_.vx = vx;
    target_.vy = vy;
    target_.w = w;
  }

  /**
   * @brief 启用或禁用控制器（切换有力无力）
   */
  void Enable(bool enable) { enabled_ = enable; }

  // getters
  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  bool enabled_{false};                         ///< 有力/无力？
  rm::modules::TriSteeringChassis kinematics_;  ///< 底盘运动学模型
  struct {
    rm::modules::PID f_wheel{50.f, 0.f, 0.f, 15000.f, 0.f};       ///< 前轮速度环
    rm::modules::PID l_wheel{50.f, 0.f, 0.f, 15000.f, 0.f};       ///< 左后轮速度环
    rm::modules::PID r_wheel{50.f, 0.f, 0.f, 15000.f, 0.f};       ///< 右后轮速度环
    rm::modules::PID f_steer{20000.f, 0.f, 200.f, 20000.f, 0.f};  ///< 前舵 位置速度并联
    rm::modules::PID l_steer{20000.f, 0.f, 200.f, 20000.f, 0.f};  ///< 左后舵 位置速度并联
    rm::modules::PID r_steer{20000.f, 0.f, 200.f, 20000.f, 0.f};  ///< 右后舵 位置速度并联
  } pid_;

  struct {
    float front_wheel_speed;     ///< 前轮速度
    float lr_wheel_speed;        ///< 左后轮速度
    float rr_wheel_speed;        ///< 右后轮速度
    float front_steer_position;  ///< 前舵位置
    float lr_steer_position;     ///< 左后舵位置
    float rr_steer_position;     ///< 右后舵位置
    float front_steer_speed;     ///< 前舵速度
    float lr_steer_speed;        ///< 左后舵速度
    float rr_steer_speed;        ///< 右后舵速度
  } state_{};                    ///< 当前状态
  struct {
    float vx;   ///< 目标线速度 x 方向
    float vy;   ///< 目标线速度 y 方向
    float w;    ///< 目标角速度
  } target_{};  ///< 目标状态
  struct {
    float front_wheel;  ///< 前轮控制输出
    float lr_wheel;     ///< 左后轮控制输出
    float rr_wheel;     ///< 右后轮控制输出
    float front_steer;  ///< 前舵控制输出
    float lr_steer;     ///< 左后舵控制输出
    float rr_steer;     ///< 右后舵控制输出
  } output_{};          ///< 控制输出
};