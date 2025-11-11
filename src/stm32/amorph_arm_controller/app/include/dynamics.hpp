
#pragma once

#include <Eigen/Dense>
#include <arm_math.h>

/**
 * @brief   机械臂的动力学模型，可以用这个类计算正动力学，或者转换成状态空间形式的模型
 */
class DynamicModel {
 public:
  using Vec3 = Eigen::Vector3f;
  using Mat3x3 = Eigen::Matrix3f;
  DynamicModel() = default;

  void Relinearize(const Vec3& q, const Vec3& qd, bool ignore_velocity = false) {
    const float q2 = q(1);
    const float q3 = q(2);
    const float dq1 = qd(0);
    const float dq2 = qd(1);
    const float dq3 = qd(2);

    const float cos_q2 = arm_cos_f32(q2);
    const float cos_q2_q3 = arm_cos_f32(q2 + q3);

    if (!ignore_velocity) {
      const float sin_q2_q3 = arm_sin_f32(q2 + q3);
      const float sin_2q2 = arm_sin_f32(2 * q2);
      const float cos_q3 = arm_cos_f32(q3);
      const float sin_q3 = arm_sin_f32(q3);
      const float sin_2q2_q3 = arm_sin_f32(2 * q2 + q3);
      const float sin_2q2_2q3 = arm_sin_f32(2 * q2 + 2 * q3);
      m_.setZero();
      m_ << i1 + a1 * cos_q2 * cos_q2 + a2 * cos_q2_q3 * cos_q2_q3 + 2 * a3 * cos_q2 * cos_q2_q3, 0, 0, 0,
          i2 + a1 + a2 + 2 * a3 * cos_q3, a2 + a3 * cos_q3, 0, a2 + a3 * cos_q3, i3 + a2;

      n_.setZero();
      n_ << -0.5 * a1 * dq2 * sin_2q2 - 0.5 * a2 * (dq2 + dq3) * sin_2q2_2q3 - a3 * dq2 * sin_2q2_q3 -
                a3 * dq3 * cos_q2 * sin_q2_q3,
          -0.5 * a1 * dq1 * sin_2q2 - 0.5 * a2 * dq1 * sin_2q2_2q3 - a3 * dq1 * sin_2q2_q3,
          -0.5 * a1 * dq1 * sin_2q2_2q3 - a3 * dq1 * cos_q2 * sin_q2_q3,
          0.5 * a1 * dq1 * sin_2q2 + 0.5 * a2 * dq1 * sin_2q2_2q3 + a3 * dq1 * sin_2q2_q3, -a3 * dq3 * sin_q3,
          -a3 * (dq2 + dq3) * sin_q3, 0.5 * a1 * dq1 * sin_2q2_2q3 + a3 * dq1 * cos_q2 * sin_q2_q3, -a3 * dq2 * sin_q3,
          0;
    }

    g_.setZero();
    g_ << 0, b1 * cos_q2 + b2 * cos_q2_q3, b2 * cos_q2_q3;
  }

  Vec3 Inverse(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool relinearize = true) {
    if (relinearize) {
      Relinearize(q, qd);
    }
    return m_ * qdd + n_ * qd + g_;
  }

  // 只计算重力项，用于重力补偿（关节位置保持/悬停）
  Vec3 Inverse(const Vec3& q, bool relinearize = true) {
    if (relinearize) {
      Relinearize(q, Vec3::Zero(), true);
    }
    return g_;
  }

  const auto& StateSpaceModel() {
    const Mat3x3 zeros = Mat3x3::Zero();
    const Mat3x3 eye = Mat3x3::Identity();

    ss_model_.A.setZero();
    ss_model_.A << zeros, -eye, zeros, -m_.inverse() * n_;

    ss_model_.B.setZero();
    ss_model_.B << zeros, eye;
    return ss_model_;
  }

 private:
  struct {
    Eigen::Matrix<float, 6, 6> A;
    Eigen::Matrix<float, 6, 3> B;
    // C = eye(6)
  } ss_model_;
  Mat3x3 m_;
  Mat3x3 n_;
  Vec3 g_;

 private:
  static constexpr float l2 = 0.1595f;  ///< 第二节连杆(pitch1)长度，单位m
  static constexpr float l3 = 0.158f;

  static constexpr float i1 = 0.2473f;  ///< 第一节连杆(yaw)的转动惯量，单位kg·m^2
  static constexpr float i2 = 0.1246f;  ///< 第二节连杆(pitch1)的转动惯量，单位kg·m^2
  static constexpr float i3 = 0.0402f;  ///< 第三节连杆(pitch2)的转动惯量，单位kg·m^2

  static constexpr float m2 = 0.37f + 0.075f;   ///< 第二节连杆(pitch1)质量，单位kg
  static constexpr float m3 = 0.321f + 0.075f;  ///< 第三节连杆(pitch2)质量，单位kg

  static constexpr float r2 = l2 / 2.f + (l2 / 2.f * 0.85f);  // 0.097f;  ///< 第二节连杆(pitch1)质心到关节轴距离，单位m
  static constexpr float r3 =
      l3 / 2.f + (l3 / 2.f * 0.21f);  // 0.150f;    ///< 第三节连杆(pitch2)质心到关节轴距离，单位m

  static constexpr float g0 = 9.8f;  ///< 重力加速度，单位m/s^2

 private:
  static constexpr float a1 = m2 * r2 * r2 + m3 * l2 * l2;
  static constexpr float a2 = m3 * r3 * r3;
  static constexpr float a3 = m3 * r3 * l2;
  static constexpr float b1 = (m2 * r2 + m3 * l2) * g0;
  static constexpr float b2 = m3 * r3 * g0;
};