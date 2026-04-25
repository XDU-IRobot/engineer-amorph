#pragma once

#include <Eigen/Dense>
#include <arm_math.h>

// tau_g[0] = (b1*cos(q2) + b2*cos(q2 + q3))*(gx*sin(q1) - gy*cos(q1));
// tau_g[1] = b1*(gx*sin(q2)*cos(q1) + gy*sin(q1)*sin(q2) - gz*cos(q2)) + b2*(gx*sin(q2 + q3)*cos(q1) +
// gy*sin(q1)*sin(q2 + q3) - gz*cos(q2 + q3)); tau_g[2] = b2*(gx*sin(q2 + q3)*cos(q1) + gy*sin(q1)*sin(q2 + q3) -
// gz*cos(q2 + q3));

// 重力补偿+平动加速度补偿
class ArmDynamics {
 public:
  ArmDynamics() = default;
  ArmDynamics(float b1, float b2) : b1_(b1), b2_(b2) {}

  Eigen::Vector3f Compensate(const Eigen::Vector3f& q, const Eigen::Vector3f& acc) const {
    const float q1 = q(0);
    const float q2 = q(1);
    const float q3 = q(2);
    const float gx = acc(0);
    const float gy = acc(1);
    const float gz = acc(2);

    const float cos_q1 = arm_cos_f32(q1);
    const float sin_q1 = arm_sin_f32(q1);
    const float cos_q2 = arm_cos_f32(q2);
    const float sin_q2 = arm_sin_f32(q2);
    const float cos_q2_q3 = arm_cos_f32(q2 + q3);
    const float sin_q2_q3 = arm_sin_f32(q2 + q3);

    Eigen::Vector3f tau_g;
    tau_g(0) = (b1_ * cos_q2 + b2_ * cos_q2_q3) * (gx * sin_q1 - gy * cos_q1);
    tau_g(1) = b1_ * (gx * sin_q2 * cos_q1 + gy * sin_q1 * sin_q2 - gz * cos_q2) +
               b2_ * (gx * sin_q2_q3 * cos_q1 + gy * sin_q1 * sin_q2_q3 - gz * cos_q2_q3);
    tau_g(2) = b2_ * (gx * sin_q2_q3 * cos_q1 + gy * sin_q1 * sin_q2_q3 - gz * cos_q2_q3);

    return tau_g;
  }

 private:
  float b1_{0.f};
  float b2_{0.f};
};
