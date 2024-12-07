
#ifndef CHASSIS_FK_HPP
#define CHASSIS_FK_HPP

#include <cmath>

class ChassisFk {
 public:
  inline ChassisFk(float wheel_base, float track) : wheel_base_(wheel_base), track_(track) {}
  void Forward(float vx, float vy, float w) {
    wheel_speeds_.l_front = vy + vx + (wheel_base_ + track_) * w;
    wheel_speeds_.r_front = vy - vx - (wheel_base_ + track_) * w;
    wheel_speeds_.l_rear = vy - vx + (wheel_base_ + track_) * w;
    wheel_speeds_.r_rear = vy + vx - (wheel_base_ + track_) * w;
    wheel_speeds_.l_mid = vy + kWRatio * w;
    wheel_speeds_.r_mid = vy - kWRatio * w;
  }

  struct {
    float l_front;
    float r_front;
    float l_mid;
    float r_mid;
    float l_rear;
    float r_rear;
  } wheel_speeds_{};

 private:
  float wheel_base_{};
  float track_{};
  const float kWRatio = std::cos(std::atan2(wheel_base_, track_));
};

#endif  // CHASSIS_FK_HPP
