#pragma once

#include <librm.hpp>

/**
 * @brief 三舵轮底盘控制器
 */
class TriSteerChassisController {
 public:
  explicit TriSteerChassisController(const ChassisConfig &config)
      : kinematics_(config.kine_base_len, config.kine_height) {
    pid_.f_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
    pid_.l_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
    pid_.r_steer.SetCircular(true).SetCircularCycle(M_PI * 2.0f).SetDiffLpfAlpha(0.9f);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float front_steer_position, float front_steer_speed,                        //
              float lr_steer_position, float lr_steer_speed,                              //
              float rr_steer_position, float rr_steer_speed,                              //
              float front_wheel_speed, float lr_wheel_speed, float rr_wheel_speed,        //
              float front_wheel_current, float lr_wheel_current, float rr_wheel_current,  //
              float front_steer_current, float lr_steer_current, float rr_steer_current,  //
              float gyro_z,                                                               //
              bool move_gently,                                                           //
              float dt = 1.f) {
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

    float vx_cmd = target_.vx;
    float vy_cmd = target_.vy;
    float w_cmd = target_.w;
    // 限制加速度，防止轮子打滑
    if (move_gently) {
      pid_.gyro_z.Clear();
      pid_.gyro_z.SetKp(0.4f);
      if (!moving_gently_) {
        pid_.gyro_z.Clear();
        pid_.gyro_z.SetKp(0.4f);
        vx_limiter_.ResetAt(0.f);
        vy_limiter_.ResetAt(0.f);
        w_limiter_.ResetAt(0.f);
        moving_gently_ = true;
      }

      // 如果命令为0就绕过限制器直接把速度命令设为0，保证刹车时响应迅速
      using rm::modules::IsNear;
      if (IsNear(target_.vx, 0.f, 1e-6)) {
        vx_limiter_.ResetAt(0.f);
      }
      if (IsNear(target_.vy, 0.f, 1e-6)) {
        vy_limiter_.ResetAt(0.f);
      }
      if (IsNear(target_.w, 0.f, 1e-6)) {
        w_limiter_.ResetAt(0.f);
      }
      vx_limiter_.SetTarget(target_.vx);
      vy_limiter_.SetTarget(target_.vy);
      w_limiter_.SetTarget(target_.w);
      vx_limiter_.Update(0.002f);
      vy_limiter_.Update(0.002f);
      w_limiter_.Update(0.002f);
      vx_cmd = vx_limiter_.current_position();
      vy_cmd = vy_limiter_.current_position();
      w_cmd = w_limiter_.current_position();
    } else {
      moving_gently_ = false;
    }

    if (move_gently) {
      // 低速模式打乱舵角的情况下，只有命令大于一定值才启用陀螺仪反馈，否则静止情况下舵会试图修正方向一直震动
      if (rm::modules::IsNear(w_cmd, 0.f, 0.05)) {
        pid_.gyro_z.Update(w_cmd, 0, dt);
      } else {
        pid_.gyro_z.Update(w_cmd, -gyro_z, dt);
      }
    } else {
      pid_.gyro_z.Update(w_cmd, -gyro_z, dt);
    }
    const float w_cmd_pid = pid_.gyro_z.out() * 2000;

    // 运动学解算
    const auto fk_result = kinematics_.Forward(vx_cmd, vy_cmd, w_cmd_pid, state_.front_steer_position,
                                               state_.lr_steer_position, state_.rr_steer_position, !move_gently);

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

    // 后处理：计算期望舵角和实际舵角的差值，两者误差过大，对应的轮速就越小，防止轮子朝错误方向发力，浪费能量
    const float f_steer_error =
        std::fabs(rm::modules::Wrap(fk_result.front_steer_position - state_.front_steer_position, -M_PI, M_PI)) / M_PI;
    const float lr_steer_error =
        std::fabs(rm::modules::Wrap(fk_result.lr_steer_position - state_.lr_steer_position, -M_PI, M_PI)) / M_PI;
    const float rr_steer_error =
        std::fabs(rm::modules::Wrap(fk_result.rr_steer_position - state_.rr_steer_position, -M_PI, M_PI)) / M_PI;
    // 用一种类sigmoid函数处理舵和轮的功率分配，k控制中间部分的陡峭程度，c控制中间陡峭部分向左向右偏移
    // f(x, k, c) = \frac{1 - e^{-kx}}{1 + e^{-k(x-c)}} \cdot \frac{1 + e^{-k(1-c)}}{1 - e^{-k}}
    // https://www.desmos.com/calculator/cunuqompad
    const static auto sigmoid_mod = [](float x, float k, float c) -> float {
      // 当 k 极小时，退化为线性函数 y = x，同时避免除以 0
      if (k < 1e-5f) {
        return x;
      }

      const float exp_nkx = std::exp(-k * x);  // e^(-kx)
      const float exp_kc = std::exp(k * c);    // e^(kc)
      const float exp_nk = std::exp(-k);       // e^(-k)

      const float term1 = (1.0f - exp_nkx) / (1.0f + exp_nkx * exp_kc);
      const float term2 = (1.0f + exp_nk * exp_kc) / (1.0f - exp_nk);

      return term1 * term2;
    };
    constexpr float K = 8.f;
    constexpr float C = 0.3f;
    const float f_steer_decay = 1 - sigmoid_mod(f_steer_error, K, C);
    const float lr_steer_decay = 1 - sigmoid_mod(lr_steer_error, K, C);
    const float rr_steer_decay = 1 - sigmoid_mod(rr_steer_error, K, C);
    // output_.front_wheel *= f_steer_decay;
    // output_.lr_wheel *= lr_steer_decay;
    // output_.rr_wheel *= rr_steer_decay;

    // 功率限制
    power_info_.steer_budget_ratio = std::clamp(
        (sigmoid_mod(f_steer_error, K, C) + sigmoid_mod(lr_steer_error, K, C) + sigmoid_mod(rr_steer_error, K, C)) /
            3.f * 1.7f,
        0.f, 1.f);
    power_info_.wheel_budget_ratio = 1.f - (sigmoid_mod(f_steer_error, K, C) + sigmoid_mod(lr_steer_error, K, C) +
                                            sigmoid_mod(rr_steer_error, K, C)) /
                                               3.f;
    constexpr float kTargetPowerLimit = 90.f;
    {  // 轮
      const float pid_outputs[3] = {output_.front_wheel, output_.lr_wheel, output_.rr_wheel};
      float pid_outputs_limited[3];
      power_info_.wheel_motor_states = {
          {{state_.front_wheel_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[0], front_wheel_current},
           {state_.lr_wheel_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[1], lr_wheel_current},
           {state_.rr_wheel_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[2], rr_wheel_current}}};
      wheel_power_model_.DistributePower(power_info_.wheel_motor_states, pid_outputs,
                                         kTargetPowerLimit * power_info_.wheel_budget_ratio, pid_outputs_limited);
      output_.front_wheel = pid_outputs_limited[0];
      output_.lr_wheel = pid_outputs_limited[1];
      output_.rr_wheel = pid_outputs_limited[2];
      power_info_.wheel_motor_states[0].give_current = output_.front_wheel;
      power_info_.wheel_motor_states[1].give_current = output_.lr_wheel;
      power_info_.wheel_motor_states[2].give_current = output_.rr_wheel;
      power_info_.estimated_wheel_power =
          wheel_power_model_.CalculatePower(power_info_.wheel_motor_states[0]).total_power +  //
          wheel_power_model_.CalculatePower(power_info_.wheel_motor_states[1]).total_power +  //
          wheel_power_model_.CalculatePower(power_info_.wheel_motor_states[2]).total_power;
    }
    {  // 舵
      const float pid_outputs[3] = {output_.front_steer, output_.lr_steer, output_.rr_steer};
      float pid_outputs_limited[3];
      power_info_.steer_motor_states = {
          {{state_.front_steer_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[0], front_steer_current},
           {state_.lr_steer_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[1], lr_steer_current},
           {state_.rr_steer_speed / 2.0f / (float)M_PI * 60.0f, pid_outputs[2], rr_steer_current}}};
      // steer_power_model_.DistributePower(power_info_.steer_motor_states, pid_outputs, 120, pid_outputs_limited);
      // output_.front_steer *= power_info_.steer_budget_ratio;
      // output_.lr_steer *= power_info_.steer_budget_ratio;
      // output_.rr_steer *= power_info_.steer_budget_ratio;
      power_info_.steer_motor_states[0].give_current = output_.front_steer;
      power_info_.steer_motor_states[1].give_current = output_.lr_steer;
      power_info_.steer_motor_states[2].give_current = output_.rr_steer;
      power_info_.estimated_steer_power =
          wheel_power_model_.CalculatePower(power_info_.steer_motor_states[0]).total_power +  //
          wheel_power_model_.CalculatePower(power_info_.steer_motor_states[1]).total_power +  //
          wheel_power_model_.CalculatePower(power_info_.steer_motor_states[2]).total_power;
    }
    power_info_.estimated_total_power = power_info_.estimated_steer_power + power_info_.estimated_wheel_power;
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
  bool moving_gently() const { return moving_gently_; }

 private:
  bool enabled_{false};                         ///< 有力/无力？
  rm::modules::TriSteeringChassis kinematics_;  ///< 底盘运动学模型
  struct {
    rm::modules::PID f_wheel{75.f, 0.f, 0.f, 15000.f, 0.f};       ///< 前轮速度环
    rm::modules::PID l_wheel{65.f, 0.f, 0.f, 15000.f, 0.f};       ///< 左后轮速度环
    rm::modules::PID r_wheel{65.f, 0.f, 0.f, 15000.f, 0.f};       ///< 右后轮速度环
    rm::modules::PID f_steer{17000.f, 0.f, 360.f, 16000.f, 0.f};  ///< 前舵 位置速度并联
    rm::modules::PID l_steer{17000.f, 0.f, 360.f, 16000.f, 0.f};  ///< 左后舵 位置速度并联
    rm::modules::PID r_steer{17000.f, 0.f, 360.f, 16000.f, 0.f};  ///< 右后舵 位置速度并联
    rm::modules::PID gyro_z{0.4f, 0.f, 0.f, 100.f, 0.f};          ///<
  } pid_;

  rm::modules::TrajectoryLimiter       //
      vx_limiter_{2000.f, 1000000.f},  //
      vy_limiter_{2000.f, 1000000.f},  //
      w_limiter_{5.f, 1000000.f};

  rm::modules::MotorPowerModel wheel_power_model_{rm::modules::MotorPowerModel::kM3508};
  rm::modules::MotorPowerModel steer_power_model_{rm::modules::MotorPowerModel::kGM6020Current};
  struct {
    float wheel_budget_ratio{};
    float steer_budget_ratio{};
    std::array<rm::modules::MotorPowerModel::MotorState, 3> steer_motor_states;
    std::array<rm::modules::MotorPowerModel::MotorState, 3> wheel_motor_states;
    float estimated_steer_power{};
    float estimated_wheel_power{};
    float estimated_total_power{};
  } power_info_;

  bool moving_gently_{false};

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