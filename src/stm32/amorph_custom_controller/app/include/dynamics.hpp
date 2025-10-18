
#pragma once

#include <Eigen/Dense>

/**
 * @brief   机械臂的动力学模型，可以用这个类计算正动力学，或者转换成状态空间形式的模型
 */
class DynamicModel {
 public:
  using Vec3 = Eigen::Vector3f;
  using Mat3x3 = Eigen::Matrix3f;
  DynamicModel() = default;

  void Relinearize(const Vec3& q, const Vec3& qd) {
    const float q2 = q(1);
    const float q3 = q(2);
    const float dq1 = qd(0);
    const float dq2 = qd(1);
    const float dq3 = qd(2);

    m_.setZero();
    m_ << i1 + a1 * std::cos(q2) * std::cos(q2) + a2 * std::cos(q2 + q3) * std::cos(q2 + q3) +
              2 * a3 * std::cos(q2) * std::cos(q2 + q3),
        0, 0, 0, i2 + a1 + a2 + 2 * a3 * std::cos(q3), a2 + a3 * std::cos(q3), 0, a2 + a3 * std::cos(q3), i3 + a2;

    n_.setZero();
    n_ << -0.5 * a1 * dq2 * std::sin(2 * q2) - 0.5 * a2 * (dq2 + dq3) * std::sin(2 * q2 + 2 * q3) -
              a3 * dq2 * std::sin(2 * q2 + q3) - a3 * dq3 * std::cos(q2) * std::sin(q2 + q3),
        -0.5 * a1 * dq1 * std::sin(2 * q2) - 0.5 * a2 * dq1 * std::sin(2 * q2 + 2 * q3) -
            a3 * dq1 * std::sin(2 * q2 + q3),
        -0.5 * a1 * dq1 * std::sin(2 * q2 + 2 * q3) - a3 * dq1 * std::cos(q2) * std::sin(q2 + q3),
        0.5 * a1 * dq1 * std::sin(2 * q2) + 0.5 * a2 * dq1 * std::sin(2 * q2 + 2 * q3) +
            a3 * dq1 * std::sin(2 * q2 + q3),
        -a3 * dq3 * std::sin(q3), -a3 * (dq2 + dq3) * std::sin(q3),
        0.5 * a1 * dq1 * std::sin(2 * q2 + 2 * q3) + a3 * dq1 * std::cos(q2) * std::sin(q2 + q3),
        -a3 * dq2 * std::sin(q3), 0;

    g_.setZero();
    g_ << 0, b1 * std::cos(q2) + b2 * std::cos(q2 + q3), b2 * std::cos(q2 + q3);
  }

  Vec3 ForwardDynamics(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool relinearize = true) {
    if (relinearize) {
      Relinearize(q, qd);
    }
    return m_ * qdd + n_ * qd + g_;
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
  static constexpr float l2 = 0.34f;
  static constexpr float l3 = 0.37f;

  static constexpr float i1 = 0.2473f;
  static constexpr float i2 = 0.1246f;
  static constexpr float i3 = 0.0402f;

  static constexpr float m2 = 1.84f;
  static constexpr float m3 = 3.46f;

  static constexpr float r2 = 0.097f;
  static constexpr float r3 = 0.150f;

  static constexpr float g0 = 9.8f;

  static constexpr float a1 = m2 * r2 * r2 + m3 * l2 * l2;
  static constexpr float a2 = m3 * r3 * r3;
  static constexpr float a3 = m3 * r3 * l2;
  static constexpr float b1 = (m2 * r2 + m3 * l2) * g0;
  static constexpr float b2 = m3 * r3 * g0;
};