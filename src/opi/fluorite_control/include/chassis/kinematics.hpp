
#ifndef CHASSIS_KINEMATICS_HPP
#define CHASSIS_KINEMATICS_HPP

#include <librm.hpp>

class ChassisFk : public rm::modules::algorithm::SteeringChassis {
 public:
  inline ChassisFk(float chassis_radius) : rm::modules::algorithm::SteeringChassis(chassis_radius) {}
  inline void operator()(float vx, float vy, float w, float current_l_angle, float current_r_angle,
                         float current_f_angle, float current_b_angle) {
    using namespace rm::modules::algorithm::utils;
    Forward(vx, vy, w);

    // 依次计算每个舵的目标角度和目前角度的差值，如果差值大于90度，就把目标舵角加180度，轮速取反
    if (LoopConstrain(current_f_angle - front_steer_angle_, -M_PI, M_PI) > M_PI / 2) {
      front_steer_angle_ = LoopConstrain(front_steer_angle_ + M_PI, 0, 2 * M_PI);
      front_wheel_speed_ = -front_wheel_speed_;
    }
    if (LoopConstrain(current_b_angle - back_steer_angle_, -M_PI, M_PI) > M_PI / 2) {
      back_steer_angle_ = LoopConstrain(back_steer_angle_ + M_PI, 0, 2 * M_PI);
      back_wheel_speed_ = -back_wheel_speed_;
    }
    if (LoopConstrain(current_l_angle - left_steer_angle_, -M_PI, M_PI) > M_PI / 2) {
      left_steer_angle_ = LoopConstrain(left_steer_angle_ + M_PI, 0, 2 * M_PI);
      left_wheel_speed_ = -left_wheel_speed_;
    }
    if (LoopConstrain(current_r_angle - right_steer_angle_, -M_PI, M_PI) > M_PI / 2) {
      right_steer_angle_ = LoopConstrain(right_steer_angle_ + M_PI, 0, 2 * M_PI);
      right_wheel_speed_ = -right_wheel_speed_;
    }
  }
};

#endif  // CHASSIS_KINEMATICS_HPP
