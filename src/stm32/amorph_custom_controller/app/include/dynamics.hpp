#pragma once

#include <Eigen/Dense>

#include <cmath>

// tau_g[0] = (b1*cos(q2) + b2*cos(q2 + q3))*(gx*sin(q1) - gy*cos(q1));
// tau_g[1] = b1*(gx*sin(q2)*cos(q1) + gy*sin(q1)*sin(q2) - gz*cos(q2)) + b2*(gx*sin(q2 + q3)*cos(q1) +
// gy*sin(q1)*sin(q2 + q3) - gz*cos(q2 + q3)); tau_g[2] = b2*(gx*sin(q2 + q3)*cos(q1) + gy*sin(q1)*sin(q2 + q3) -
// gz*cos(q2 + q3));

// 重力补偿+平动加速度补偿
class ArmDynamics {
 public:
  struct Params {
    float b1;
    float b2;
  };

  ArmDynamics() = default;
  explicit ArmDynamics(Params params) : params_(std::move(params)) {}

  Eigen::Vector3f Compensate(const Eigen::Vector3f& q, const Eigen::Vector3f& acc) const {
    const float q1 = q(0);
    const float q2 = q(1);
    const float q3 = q(2);
    const float gx = acc(0);
    const float gy = acc(1);
    const float gz = acc(2);

    const float cos_q1 = std::cos(q1);
    const float sin_q1 = std::sin(q1);
    const float cos_q2 = std::cos(q2);
    const float sin_q2 = std::sin(q2);
    const float cos_q2_q3 = std::cos(q2 + q3);
    const float sin_q2_q3 = std::sin(q2 + q3);

    Eigen::Vector3f tau_g;
    tau_g(0) = (params_.b1 * cos_q2 + params_.b2 * cos_q2_q3) * (gx * sin_q1 - gy * cos_q1);
    tau_g(1) = params_.b1 * (gx * sin_q2 * cos_q1 + gy * sin_q1 * sin_q2 - gz * cos_q2) +
               params_.b2 * (gx * sin_q2_q3 * cos_q1 + gy * sin_q1 * sin_q2_q3 - gz * cos_q2_q3);
    tau_g(2) = params_.b2 * (gx * sin_q2_q3 * cos_q1 + gy * sin_q1 * sin_q2_q3 - gz * cos_q2_q3);

    return tau_g;
  }


  Eigen::Vector3f ComputeTorque(const Eigen::Vector3f& q, const Eigen::Vector3f& qd, const Eigen::Vector3f& qdd,
                                const Eigen::Vector3f& acc) {
    const float q1 = q(0);
    const float q2 = q(1);
    const float q3 = q(2);
    const float dq1 = qd(0);
    const float dq2 = qd(1);
    const float dq3 = qd(2);
    const float gx = acc(0);
    const float gy = acc(1);
    const float gz = acc(2);

    // ---- 预计算三角函数 ----
    const float s1 = sinf(q1), c1 = cosf(q1);
    const float s2 = sinf(q2), c2 = cosf(q2);
    const float s3 = sinf(q3), c3 = cosf(q3);
    const float s23 = sinf(q2 + q3), c23 = cosf(q2 + q3);
    const float s2x2 = sinf(2 * q2), c2x2 = cosf(2 * q2);
    const float s2x2q3 = sinf(2 * q2 + q3), c2x2q3 = cosf(2 * q2 + q3);
    const float s2x2q3x2 = sinf(2 * q2 + 2 * q3), c2x2q3x2 = cosf(2 * q2 + 2 * q3);

    // ---- 质量矩阵 M(q) (对称) ----
    Eigen::Matrix3f M;
    M(0, 0) = 0.03035980234115538 * c2x2 + 0.026252138880000012 * c2x2q3 + 0.0097449355000000012 * c2x2q3x2 +
              0.026252138880000012 * c3 + 0.040210758341155381;
    M(0, 1) = 0;
    M(1, 0) = M(0, 1);  // 对称
    M(0, 2) = 0;
    M(2, 0) = M(0, 2);  // 对称
    M(1, 1) = 0.052504277760000009 * c3 + 0.08031549618231075;
    M(1, 2) = 0.026252138880000005 * c3 + 0.019542679;
    M(2, 1) = M(1, 2);  // 对称
    M(2, 2) = 0.019542679;

    // ---- 科里奥利+离心力向量 C(q, dq) * dq ----
    Eigen::Vector3f C_dq;
    C_dq(0) = -1.0 * dq1 *
              (dq2 * (0.06071960468231076 * s2x2 + 0.052504277760000009 * s2x2q3 + 0.019489871000000002 * s2x2q3x2) +
               dq3 * (0.026252138880000005 * s2x2q3 + 0.019489871000000002 * s2x2q3x2 + 0.026252138880000005 * s3));
    C_dq(1) =
        dq1 * dq1 * (0.03035980234115538 * s2x2 + 0.026252138880000005 * s2x2q3 + 0.0097449355000000012 * s2x2q3x2) -
        0.026252138880000005 * dq2 * dq3 * s3 - 0.026252138880000005 * dq3 * s3 * (dq2 + dq3);
    C_dq(2) =
        dq1 * dq1 * (0.013126069440000002 * s2x2q3 + 0.0097449355000000012 * s2x2q3x2 + 0.013126069440000002 * s3) +
        0.026252138880000005 * dq2 * dq2 * s3;

    // ---- 重力向量 G(q) ----
    Eigen::Vector3f G;
    G(0) = -(0.26455448800000003 * c2 + 0.10944000000000001 * c23) * (c1 * gy - gx * s1);
    G(1) = 0.26455448800000003 * c1 * gx * s2 + 0.10944000000000001 * c1 * gx * s23 - 0.26455448800000003 * c2 * gz -
           0.10944000000000001 * c23 * gz + 0.26455448800000003 * gy * s1 * s2 + 0.10944000000000001 * gy * s1 * s23;
    G(2) = 0.10944000000000001 * c1 * gx * s23 - 0.10944000000000001 * c23 * gz + 0.10944000000000001 * gy * s1 * s23;

    // ---- 组装：tau = M*ddq + C_dq + G ----
    return M * qdd + C_dq + G;
  }

 private:
  Params params_;
};
