

#include <cmath>

#include <Eigen/Dense>

namespace fluorite {

class StateSpaceModel {
 public:
  void UpdateParameters(double q1,  ///< yaw
                        double q2,  ///< pitch_1
                        double q3,  ///< pitch_2
                        double dq1, double dq2, double dq3) {
    this->q1_ = q1;
    this->q2_ = q2;
    this->q3_ = q3;
    this->dq1_ = dq1;
    this->dq2_ = dq2;
    this->dq3_ = dq3;
    ComputeMN();
    ComputeAB();
  }

  const Eigen::MatrixXd& A() const { return A_; }

  const Eigen::MatrixXd& B() const { return B_; }

 private:
  void ComputeMN() {
    M_.resize(3, 3);
    M_ << 0.07785 * std::cos(q2_ + q3_) * std::cos(q2_ + q3_) + 0.4173 * std::cos(q2_) * std::cos(q2_) +
              0.3529 * std::cos(q2_ + q3_) * std::cos(q2_) + 0.2473,
        0, 0, 0, 0.3529 * std::cos(q3_) + 0.6197, 0.1765 * std::cos(q3_) + 0.07785, 0, 0.1765 * std::cos(q3_) + 0.07785,
        0.1181;

    N_.resize(3, 3);
    N_ << -0.1765 * dq2_ * std::sin(2.0 * q2_ + q3_) - 0.2086 * dq2_ * std::sin(2.0 * q2_) -
              std::sin(2.0 * q2_ + 2.0 * q3_) * (0.03892 * dq2_ + 0.03892 * dq3_) -
              0.1765 * dq3_ * std::sin(q2_ + q3_) * std::cos(q2_),
        -0.1765 * dq1_ * std::sin(2.0 * q2_ + q3_) - 0.2086 * dq1_ * std::sin(2.0 * q2_) -
            0.03892 * dq1_ * std::sin(2.0 * q2_ + 2.0 * q3_),
        -0.2086 * dq1_ * std::sin(2.0 * q2_ + 2.0 * q3_) - 0.1765 * dq1_ * std::sin(q2_ + q3_) * std::cos(q2_),
        0.1765 * dq1_ * std::sin(2.0 * q2_ + q3_) + 0.2086 * dq1_ * std::sin(2.0 * q2_) +
            0.03892 * dq1_ * std::sin(2.0 * q2_ + 2.0 * q3_),
        -0.1765 * dq3_ * std::sin(q3_), -std::sin(q3_) * (0.1765 * dq2_ + 0.1765 * dq3_),
        0.2086 * dq1_ * std::sin(2.0 * q2_ + 2.0 * q3_) + 0.1765 * dq1_ * std::sin(q2_ + q3_) * std::cos(q2_),
        -0.1765 * dq2_ * std::sin(q3_), 0;
  }

  void ComputeAB() {
    int size = M_.rows();
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(size, size);
    Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(size, size);

    A_.resize(2 * size, 2 * size);
    A_ << zeros, -eye, zeros, -M_.inverse() * N_;

    B_.resize(2 * size, size);
    B_ << zeros, eye;
  }

 private:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd M_;
  Eigen::MatrixXd N_;

  double q1_, q2_, q3_, dq1_, dq2_, dq3_;
};

}  // namespace fluorite