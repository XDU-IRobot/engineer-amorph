
#include "arm/dynamics.hpp"

using Vec3 = fluorite::arm::DynamicModel::Vec3;

Vec3 fluorite::arm::DynamicModel::operator()(const Vec3& q, const Vec3& qd, bool logging) {
  Vec3 ret;
  Vec3 qdd_differential = (qd - last_qd_.second) / (rclcpp::Clock().now() - last_qd_.first).seconds();

  ret = (*this)(q, qd, qdd_differential, logging);

  last_qd_ = std::make_pair(rclcpp::Clock().now(), qd);
  return ret;
}

Vec3 fluorite::arm::DynamicModel::operator()(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool logging) {
  using rm::modules::algorithm::utils::DegToRad;

  double q2 = DegToRad(160) - q(1);
  double q3 = q(2) - DegToRad(160);
  double dq1 = qd(0), dq2 = -qd(1), dq3 = qd(2);

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
      0.5 * a1 * dq1 * std::sin(2 * q2) + 0.5 * a2 * dq1 * std::sin(2 * q2 + 2 * q3) + a3 * dq1 * std::sin(2 * q2 + q3),
      -a3 * dq3 * std::sin(q3), -a3 * (dq2 + dq3) * std::sin(q3),
      0.5 * a1 * dq1 * std::sin(2 * q2 + 2 * q3) + a3 * dq1 * std::cos(q2) * std::sin(q2 + q3),
      -a3 * dq2 * std::sin(q3), 0;

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

  return -1 * (m_ * qdd + n_ * qd + g_);
}