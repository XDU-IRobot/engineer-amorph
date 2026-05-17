#pragma once

#include <Eigen/Dense>
#include <cmath>

/// Full forward dynamics for 3-axis arm (yaw + pitch1 + pitch2).
/// τ = M(q)·ddq + c(q,dq) + G(q,acc) + τ_f(dq) + offset
///
/// Identified with physically measured d1, d2 (not free regression parameters)
/// and per-joint offsets to absorb torque sensor zero drift.
/// Validated on both excitation trajectory and quasi-static holding data.
class FullArmDynamics {
 public:
  struct Params {
    // ─── M(q) matrix coefficients ───
    // M(0,0) = c0 + c1·cos(2q2) + c2·cos(q3) + c2·cos(2q2+q3) + c3·cos(2q2+2q3)
    // M(1,1) = c4·cos(q3) + c5
    // M(1,2) = c6·cos(q3) + c7
    // M(2,2) = c7
    // M(0,1) = M(0,2) = 0
    float c0 = 0.64544134245f;
    float c1 = 0.28419701935f;
    float c2 = 0.46817925f;
    float c3 = 0.2676403455f;
    float c4 = 0.9363585f;
    float c5 = 1.7761783684f;
    float c6 = 0.46817925f;
    float c7 = 0.028087079f;

    // Derived (precomputed for speed)
    float c1_2 = 0.5683940387f;  // 2·c1
    float c2_2 = 0.9363585f;     // 2·c2
    float c3_2 = 0.535280691f;   // 2·c3
    float c4_2 = 0.9363585f;     // = c4 = 2·c2
    float c2_h = 0.234089625f;   // c2/2

    // ─── Gravity (coefficients before multiplying by acceleration) ───
    float g0 = 1.733725f;  // d1·m1 + L1·m2  (measured)
    float g1 = 1.1055f;    // d2·m2           (measured)

    // ─── Friction ───
    // J1, J2: sign model (τ_f = fc·sign(dq) + fv·dq)
    // J3: directional tanh model
    float fc1 = 0.596329f;
    float fv1 = 1.182905f;
    float fc2 = 0.849158f;
    float fv2 = 1.211189f;
    float fv3 = 1.532695f;
    float alpha3 = 27.996f;
    float fc3p = -0.042785f;
    float fc3n = 12.519656f;

    // ─── Per-joint torque sensor offsets ───
    float off1 = -0.115609f;
    float off2 = 0.625042f;
    float off3 = 8.612281f;
  };

  FullArmDynamics() = default;
  explicit FullArmDynamics(const Params& p) : params_(p) {}

  /// Mutable access to parameters for runtime tuning.
  Params params;

  /// Full forward dynamics torque.
  /// q   = [yaw, pitch1, pitch2]  (rad)
  /// dq  = joint velocities        (rad/s)
  /// ddq = joint accelerations     (rad/s²)
  /// acc = base linear acceleration in world frame (m/s²), typically [0, 0, -g] for static
  Eigen::Vector3f Forward(const Eigen::Vector3f& q, const Eigen::Vector3f& dq, const Eigen::Vector3f& ddq,
                          const Eigen::Vector3f& acc) const {
    const float s2 = std::sin(q(1));
    const float c2 = std::cos(q(1));
    const float s3 = std::sin(q(2));
    const float c3 = std::cos(q(2));
    const float s23 = std::sin(q(1) + q(2));
    const float c23 = std::cos(q(1) + q(2));
    const float s2q2 = std::sin(2.f * q(1));
    const float c2q2 = std::cos(2.f * q(1));
    const float s2q2q3 = std::sin(2.f * q(1) + q(2));
    const float c2q2q3 = std::cos(2.f * q(1) + q(2));
    const float s2q23 = std::sin(2.f * (q(1) + q(2)));
    const float c2q23 = std::cos(2.f * (q(1) + q(2)));

    const float dq0 = dq(0);
    const float dq1 = dq(1);
    const float dq2 = dq(2);

    // ─── M(q)·ddq ───
    const float m00 = params_.c0 + params_.c1 * c2q2 + params_.c2 * c3 + params_.c2 * c2q2q3 + params_.c3 * c2q23;
    const float m11 = params_.c4 * c3 + params_.c5;
    const float m12 = params_.c6 * c3 + params_.c7;
    const float m22 = params_.c7;

    Eigen::Vector3f tau;
    tau(0) = m00 * ddq(0);
    tau(1) = m11 * ddq(1) + m12 * ddq(2);
    tau(2) = m12 * ddq(1) + m22 * ddq(2);

    // ─── c(q, dq) ───
    const float dq0_sq = dq0 * dq0;
    tau(0) -= dq0 * (params_.c1_2 * dq1 * s2q2 + params_.c2_2 * dq1 * s2q2q3 + params_.c3_2 * dq1 * s2q23 +
                     params_.c2 * dq2 * s3 + params_.c2 * dq2 * s2q2q3 + params_.c3 * dq2 * s2q23);
    tau(1) += dq0_sq * (params_.c1 * s2q2 + params_.c2 * s2q2q3 + params_.c3 * s2q23) - params_.c4_2 * dq1 * dq2 * s3 -
              params_.c2 * dq2 * dq2 * s3;
    tau(2) += dq0_sq * (params_.c2_h * s3 + params_.c2_h * s2q2q3 + params_.c3 * s2q23) + params_.c2 * dq1 * dq1 * s3;

    // ─── G(q, acc) ───
    const float a0 = acc(0);
    const float a1 = acc(1);
    const float a2 = acc(2);
    const float s1 = std::sin(q(0));
    const float c1 = std::cos(q(0));

    const float g0_c = params_.g0 * c2 + params_.g1 * c23;
    tau(0) += (a0 * s1 - a1 * c1) * g0_c;
    tau(1) += a0 * (params_.g0 * s2 * c1 + params_.g1 * s23 * c1) +
              a1 * (params_.g0 * s1 * s2 + params_.g1 * s1 * s23) - a2 * g0_c;
    tau(2) += a1 * params_.g1 * s1 * s23 + a0 * params_.g1 * s23 * c1 - a2 * params_.g1 * c23;

    // ─── τ_f(dq) ───
    tau(0) += params_.fc1 * Sign(dq0) + params_.fv1 * dq0;
    tau(1) += params_.fc2 * Sign(dq1) + params_.fv2 * dq1;
    tau(2) += FrictionJ3(dq2);

    // ─── Per-joint offset ───
    tau(0) += params_.off1;
    tau(1) += params_.off2;
    tau(2) += params_.off3;

    return tau;
  }

  /// Gravity + base acceleration compensation only (no inertial or friction terms).
  Eigen::Vector3f Compensate(const Eigen::Vector3f& q, const Eigen::Vector3f& acc) const {
    const float s1 = std::sin(q(0));
    const float c1 = std::cos(q(0));
    const float s2 = std::sin(q(1));
    const float c2 = std::cos(q(1));
    const float s23 = std::sin(q(1) + q(2));
    const float c23 = std::cos(q(1) + q(2));

    const float a0 = acc(0);
    const float a1 = acc(1);
    const float a2 = acc(2);

    Eigen::Vector3f tau;
    tau(0) = (a0 * s1 - a1 * c1) * (params_.g0 * c2 + params_.g1 * c23) + params_.off1;
    tau(1) = a0 * (params_.g0 * s2 * c1 + params_.g1 * s23 * c1) + a1 * (params_.g0 * s1 * s2 + params_.g1 * s1 * s23) -
             a2 * (params_.g0 * c2 + params_.g1 * c23) + params_.off2;
    tau(2) = a1 * params_.g1 * s1 * s23 + a0 * params_.g1 * s23 * c1 - a2 * params_.g1 * c23 + params_.off3;
    return tau;
  }

 private:
  static float Sign(float x) { return (x > 0.f) ? 1.f : ((x < 0.f) ? -1.f : 0.f); }

  float FrictionJ3(float dq) const {
    if (dq > 0.f) {
      return params_.fc3p * std::tanh(params_.alpha3 * dq) + params_.fv3 * dq;
    }
    return params_.fc3n * std::tanh(params_.alpha3 * dq) + params_.fv3 * dq;
  }

  Params params_;
};
