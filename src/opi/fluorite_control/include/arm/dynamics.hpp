#ifndef ARM_DYNAMICS_HPP
#define ARM_DYNAMICS_HPP

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <utility>
#include <cmath>
#include <sstream>

#include <librm.hpp>
#include <Eigen/Dense>

#include "common/logger.hpp"

namespace fluorite {

class DynamicModel {
 public:
  // 定义类型别名
  using Vec3 = Eigen::Vector3d;
  using Mat3x3 = Eigen::Matrix3d;
  Vec3 operator()(const Vec3& q, const Vec3& qd, bool logging = false) {
    Vec3 ret;
    Vec3 qdd_differential = (qd - last_qd_.second) / (rclcpp::Clock().now() - last_qd_.first).seconds();

    ret = (*this)(q, qd, qdd_differential, logging);

    last_qd_ = std::make_pair(rclcpp::Clock().now(), qd);
    return ret;
  }

  // 动态模型计算
  Vec3 operator()(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool logging = false) {
    using rm::modules::algorithm::utils::DegToRad;

    double q2 = DegToRad(160) - q(1);
    double q3 = q(2) - DegToRad(160);
    double dq1 = qd(0), dq2 = -qd(1), dq3 = qd(2);

    // 计算惯性矩阵 M
    m_.setZero();
    m_ << i1 + a1 * std::cos(q2) * std::cos(q2) + a2 * std::cos(q2 + q3) * std::cos(q2 + q3) +
              2 * a3 * std::cos(q2) * std::cos(q2 + q3),
        0, 0, 0, i2 + a1 + a2 + 2 * a3 * std::cos(q3), a2 + a3 * std::cos(q3), 0, a2 + a3 * std::cos(q3), i3 + a2;

    // 计算科氏力和离心力矩阵 N
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

    // 计算重力向量 g
    g_.setZero();
    g_ << 0, b1 * std::cos(q2) + b2 * std::cos(q2 + q3), b2 * std::cos(q2 + q3);

    if (logging) {
      std::ostringstream oss;
      oss << "=================================================\n";
      oss << "q: " << q.transpose() << "\n"
          << "qd: " << qd.transpose() << "\n"
          << "qdd: " << qdd.transpose() << "\n";
      oss << "-------------------------------------------------\n";
      oss << "M: " << m_ << "\nN: " << n_ << "\ng: " << g_;
      info("%s", oss.str().c_str());
    }

    // 计算关节力矩 tau
    return m_ * qdd + n_ * qd + g_;
  }

 private:
  std::pair<rclcpp::Time, Vec3> last_qd_{};
  Mat3x3 m_, n_, g_;

  static constexpr double l2 = 0.34;
  static constexpr double l3 = 0.37;

  static constexpr double i1 = 0.2473;
  static constexpr double i2 = 0.1246;
  static constexpr double i3 = 0.0402;

  static constexpr double m2 = 1.84;
  static constexpr double m3 = 3.46;

  static constexpr double r2 = 0.097;
  static constexpr double r3 = 0.150;

  static constexpr double g0 = 9.8;

  static constexpr double a1 = m2 * r2 * r2 + m3 * l2 * l2;
  static constexpr double a2 = m3 * r3 * r3;
  static constexpr double a3 = m3 * r3 * l2;
  static constexpr double b1 = (m2 * r2 + m3 * l2) * g0;
  static constexpr double b2 = m3 * r3 * g0;
};

}  // namespace fluorite

#endif  // ARM_DYNAMICS_HPP