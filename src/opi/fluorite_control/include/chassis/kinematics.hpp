
#ifndef CHASSIS_KINEMATICS_HPP
#define CHASSIS_KINEMATICS_HPP

#include <librm.hpp>

class ChassisFk {
 private:
  double chassis_radius_;

 public:
  double lf_steer_position, rf_steer_position, lr_steer_position, rr_steer_position;
  double lf_wheel_speed, rf_wheel_speed, lr_wheel_speed, rr_wheel_speed;
  inline ChassisFk(float chassis_radius) : chassis_radius_(chassis_radius) {}
  inline void operator()(float vx, float vy, float w, float current_lf_angle, float current_rf_angle,
                         float current_lr_angle, float current_rr_angle) {
    using namespace rm::modules::algorithm::utils;

    // shouldn't compare floating point numbers directly but it works here
    if (vx == 0 && vy == 0 && w == 0) {
      lf_steer_position = M_PI / 4.f;
      rf_steer_position = 3 * M_PI / 4.f;
      lr_steer_position = 3 * M_PI / 4.f;
      rr_steer_position = M_PI / 4.f;
    } else {
      lf_steer_position = atan2f(vy + w * chassis_radius_ * sqrtf(2) / 2, vx + w * chassis_radius_ * sqrtf(2) / 2);
      rf_steer_position = atan2f(vy + w * chassis_radius_ * sqrtf(2) / 2, vx - w * chassis_radius_ * sqrtf(2) / 2);
      lr_steer_position = atan2f(vy - w * chassis_radius_ * sqrtf(2) / 2, vx + w * chassis_radius_ * sqrtf(2) / 2);
      rr_steer_position = atan2f(vy - w * chassis_radius_ * sqrtf(2) / 2, vx - w * chassis_radius_ * sqrtf(2) / 2);
    }

    lf_wheel_speed =
        sqrtf(pow(vy - w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx - w * chassis_radius_ * sqrtf(2) / 2, 2));
    rf_wheel_speed =
        sqrtf(pow(vy + w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx - w * chassis_radius_ * sqrtf(2) / 2, 2));
    lr_wheel_speed =
        sqrtf(pow(vy + w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx + w * chassis_radius_ * sqrtf(2) / 2, 2));
    rr_wheel_speed =
        sqrtf(pow(vy - w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx + w * chassis_radius_ * sqrtf(2) / 2, 2));

    // 依次计算每个舵的目标角度和目前角度的差值，如果差值大于90度，就把目标舵角加180度，轮速取反
    if (std::abs(LoopConstrain(current_lf_angle - lf_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      lf_steer_position = LoopConstrain(lf_steer_position + M_PI, 0, 2 * M_PI);
      lf_wheel_speed = -lf_wheel_speed;
    }
    if (std::abs(LoopConstrain(current_rf_angle - rf_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      rf_steer_position = LoopConstrain(rf_steer_position + M_PI, 0, 2 * M_PI);
      rf_wheel_speed = -rf_wheel_speed;
    }
    if (std::abs(LoopConstrain(current_lr_angle - lr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      lr_steer_position = LoopConstrain(lr_steer_position + M_PI, 0, 2 * M_PI);
      lr_wheel_speed = -lr_wheel_speed;
    }
    if (std::abs(LoopConstrain(current_rr_angle - rr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      rr_steer_position = LoopConstrain(rr_steer_position + M_PI, 0, 2 * M_PI);
      rr_wheel_speed = -rr_wheel_speed;
    }
  }
};

#endif  // CHASSIS_KINEMATICS_HPP
