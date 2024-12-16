#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <qpOASES.hpp>

using namespace qpOASES;
using namespace Eigen;

inline void demo() {
  // 系统模型
  const int n = 2;   // 状态维度
  const int m = 1;   // 控制输入维度
  const int N = 10;  // 预测时域长度

  MatrixXd A(n, n);
  A << 1, 1, 0, 1;
  MatrixXd B(n, m);
  B << 0.5, 1;

  // MPC参数
  MatrixXd Q = MatrixXd::Identity(n, n);
  MatrixXd R = MatrixXd::Identity(m, m);

  // 初始状态
  VectorXd x0(n);
  x0 << 0, 0;

  // 目标状态（阶跃目标）
  VectorXd x_ref(n);
  x_ref << 1, 1;

  // 构建QP问题
  int numVars = N * (n + m);
  int numConstr = N * n;

  MatrixXd H = MatrixXd::Zero(numVars, numVars);
  VectorXd g = VectorXd::Zero(numVars);
  MatrixXd Aeq = MatrixXd::Zero(numConstr, numVars);
  VectorXd lb = VectorXd::Constant(numVars, -1e20);
  VectorXd ub = VectorXd::Constant(numVars, 1e20);
  VectorXd lbA = VectorXd::Constant(numConstr, -1e20);
  VectorXd ubA = VectorXd::Constant(numConstr, 1e20);

  // 填充H矩阵
  for (int i = 0; i < N; ++i) {
    H.block(i * n, i * n, n, n) = Q;
    H.block(N * n + i * m, N * n + i * m, m, m) = R;
  }

  // 填充g向量
  for (int i = 0; i < N; ++i) {
    g.segment(i * n, n) = -Q * x_ref;
  }

  // 填充Aeq矩阵
  for (int i = 0; i < N; ++i) {
    Aeq.block(i * n, i * n, n, n) = MatrixXd::Identity(n, n);
    if (i > 0) {
      Aeq.block(i * n, (i - 1) * n, n, n) = -A;
      Aeq.block(i * n, N * n + (i - 1) * m, n, m) = -B;
    }
  }

  // 初始状态约束
  lbA.head(n) = x0;
  ubA.head(n) = x0;

  // 使用qpOASES求解QP问题
  QProblem qp(numVars, numConstr);
  Options options;
  qp.setOptions(options);

  int nWSR = 100;
  qp.init(H.data(), g.data(), Aeq.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);

  VectorXd xOpt(numVars);
  qp.getPrimalSolution(xOpt.data());

  // 输出最优控制输入
  std::cout << "Optimal control inputs:" << std::endl;
  for (int i = 0; i < N; ++i) {
    std::cout << xOpt(N * n + i * m) << std::endl;
  }

  // 输出应用控制输出后的系统状态
  VectorXd x = x0;
  std::cout << "System states after applying optimal control inputs:" << std::endl;
  for (int i = 0; i < N; ++i) {
    x = A * x + B * xOpt(N * n + i * m);
    std::cout << x.transpose() << std::endl;
  }
}
