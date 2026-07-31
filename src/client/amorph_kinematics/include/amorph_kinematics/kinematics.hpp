#pragma once

#include <Eigen/Dense>
#include <vector>

#define ABS_FN(x) std::abs(x)
#define ATAN2_FN(y, x) std::atan2(y, x)
#define COS_FN(x) std::cos(x)
#define SIN_FN(x) std::sin(x)
#define SQRT_FN(x) std::sqrt(x)

namespace amorph_kinematics {

/**
 * @brief 6-DOF anthropomorphic arm kinematics (PUMA560-style Modified DH).
 *
 * The Modified DH (Craig's convention) parameter table:
 *
 *   i | alpha_{i-1} | a_{i-1} | theta_i  | d_i
 *  ---|-------------|---------|----------|-----
 *   1 |      0      |    0    |   th1    |  0
 *   2 |    90°      |    0    |   th2    |  0
 *   3 |      0      |   a2_   |   th3    | d3_
 *   4 |    -90°     |   a3_   |   th4    | d4_
 *   5 |    90°      |    0    |   th5    |  0
 *   6 |    -90°     |    0    |   th6    |  0
 *
 * Constructor parameters:
 *   a2  – upper-arm link length
 *   a3  – elbow x-offset
 *   d3  – shoulder z-offset
 *   d4  – forearm / wrist-offset length
 */
class ArmKinematics {
 public:
  ArmKinematics(double a2 = 0.472254, double a3 = 0.0, double d3 = 0.0, double d4 = 0.45)
      : a2_(a2), a3_(a3), d3_(d3), d4_(d4) {
    // 预计算逆运动学常量
    K_ = a2_ * a2_ + a3_ * a3_ + d3_ * d3_ + d4_ * d4_;
    b_coeff_ = 4.0 * a2_ * d4_;
    offset_2a2a3_ = 2.0 * a2_ * a3_;
  }

  // ---------------------------------------------------------------------------
  // 正运动学
  // ---------------------------------------------------------------------------

  /**
   * @brief 由6个关节角计算末端执行器位姿（坐标系6）。
   * @param theta 关节角 [th1..th6]，单位为弧度。
   * @return 4x4 齐次变换矩阵 T0_6。
   */
  Eigen::Matrix4d SolveFk(const Eigen::Matrix<double, 6, 1> &theta) const {
    // 将 URDF 关节角转换为内部 MDH 约定
    const Eigen::Matrix<double, 6, 1> th = urdfToModel(theta);
    const double t1 = th(0), t2 = th(1), t3 = th(2);
    const double t4 = th(3), t5 = th(4), t6 = th(5);

    const double c1 = COS_FN(t1), s1 = SIN_FN(t1);
    const double c2 = COS_FN(t2), s2 = SIN_FN(t2);
    const double c3 = COS_FN(t3), s3 = SIN_FN(t3);
    const double c4 = COS_FN(t4), s4 = SIN_FN(t4);
    const double c5 = COS_FN(t5), s5 = SIN_FN(t5);
    const double c6 = COS_FN(t6), s6 = SIN_FN(t6);

    // ---- 旋转矩阵 (R_{i-1,i}) -----------------------------------------------
    // clang-format off
    Eigen::Matrix3d R01;
    R01 <<  c1, -s1,  0,
            s1,  c1,  0,
            0,   0,   1;
    Eigen::Matrix3d R12;
    R12 <<  c2, -s2,  0,
             0,   0, -1,
            s2,  c2,  0;
    Eigen::Matrix3d R23;
    R23 <<  c3, -s3,  0,
            s3,  c3,  0,
             0,   0,  1;
    Eigen::Matrix3d R34;
    R34 <<  c4, -s4,  0,
             0,   0,  1,
           -s4, -c4,  0;
    Eigen::Matrix3d R45;
    R45 <<  c5, -s5,  0,
             0,   0, -1,
            s5,  c5,  0;
    Eigen::Matrix3d R56;
    R56 <<  c6, -s6,  0,
             0,   0,  1,
           -s6, -c6,  0;
    // clang-format on

    // ---- 平移向量 -----------------------------------------------------------
    // p23 = [a2; 0; d3]（坐标系3原点在坐标系2中的表示）
    const Eigen::Vector3d p23(a2_, 0.0, d3_);
    // p34 = [a3; d4; 0]（坐标系4原点在坐标系3中的表示）
    const Eigen::Vector3d p34(a3_, d4_, 0.0);

    // 坐标系4原点在坐标系2中的位置
    const Eigen::Vector3d p24 = p23 + R23 * p34;
    // 末端执行器在坐标系0中的位置（腕心 = 坐标系6原点）
    const Eigen::Vector3d p = R01 * R12 * p24;

    // 坐标系6在坐标系0中的姿态
    const Eigen::Matrix3d R = R01 * R12 * R23 * R34 * R45 * R56;

    // 构建 4x4 齐次变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  // ---------------------------------------------------------------------------
  // 逆运动学
  // ---------------------------------------------------------------------------

  /**
   * @brief 对期望末端位姿求解逆运动学。
   *
   * 使用 PUMA560 构型机械臂的解析闭式解。
   * 内部自动完成去重和肘部朝上筛选，理想情况返回 2 组解（正手/反手）。
   *
   * @param T_target 期望的 4x4 齐次变换矩阵 (T0_6)。
   * @return 有效 6x1 关节角向量（URDF 约定），单位为弧度。
   */
  std::vector<Eigen::Matrix<double, 6, 1>> SolveIk(const Eigen::Matrix4d &T_target) const {
    // =======================================================================
    // 阶段 1：解析求解前三个关节角 (th3 → th2 → th1)
    // th3: 2 种取值 (肘部上下)
    // th2: 每 th3 最多 2 种 (± 分支)，取决于 a2_eq != 0
    // th1: 由 atan2 直接计算
    // =======================================================================

    constexpr double kEps = 1e-12;

    Eigen::Vector3d p = T_target.block<3, 1>(0, 3);
    Eigen::Matrix3d R06 = T_target.block<3, 3>(0, 0);

    double x = p(0), y = p(1), z = p(2);
    double r2 = x * x + y * y + z * z;

    std::vector<Eigen::Matrix<double, 6, 1>> solutions;
    solutions.reserve(8);

    // ---- 求解 theta3 -------------------------------------------------------
    double a = r2 - K_ + offset_2a2a3_;
    double c = r2 - K_ - offset_2a2a3_;
    double Del3 = b_coeff_ * b_coeff_ - 4.0 * a * c;

    if (Del3 < 0.0) {
      return solutions;  // 无解
    }

    double sqrt_Del3 = sqrt(Del3);
    double t3_1 = (-b_coeff_ + sqrt_Del3) / (2.0 * a);
    double t3_2 = (-b_coeff_ - sqrt_Del3) / (2.0 * a);

    // th3 有 2 个可能取值，每个产生 2 个 theta2 取值
    double th3_vals[2] = {2.0f * atan(t3_1), 2.0f * atan(t3_2)};

    // ---- 对每个 theta3 求解 theta2 -----------------------------------------
    // 结果：th2 矩阵 [2 x 2]，行 = theta3 下标，列 = +/- 分支
    double th2_vals[2][2];
    bool th2_valid[2][2] = {{false, false}, {false, false}};

    for (int i3 = 0; i3 < 2; ++i3) {
      double th3 = th3_vals[i3];
      double c3 = COS_FN(th3), s3 = SIN_FN(th3);

      double f1 = a3_ * c3 - d4_ * s3 + a2_;
      double f2 = a3_ * s3 + d4_ * c3;

      double Del2 = 4.0 * f1 * f1 - 4.0 * (z * z - f2 * f2);
      if (Del2 < 0.0) {
        continue;
      }
      double sqrt_Del2 = sqrt(Del2);

      double a2_eq = z + f2;
      double b2_eq = -2.0 * f1;
      (void)(z - f2);  // c2_eq，仅在公式推导中引用

      if (abs(a2_eq) > kEps) {
        double t2_1 = (-b2_eq + sqrt_Del2) / (2.0 * a2_eq);
        double t2_2 = (-b2_eq - sqrt_Del2) / (2.0 * a2_eq);
        th2_vals[i3][0] = 2.0 * atan(t2_1);
        th2_vals[i3][1] = 2.0 * atan(t2_2);
        th2_valid[i3][0] = true;
        th2_valid[i3][1] = true;
      }
    }

    // =======================================================================
    // 阶段 2：对每组 (th3, th2) 组合求解 th1 和腕部角度 (th4/th5/th6)
    // 腕部每组合有 2 种解（正手/反手），因此最多产生 2×2×2 = 8 组解
    // =======================================================================
    for (int i3 = 0; i3 < 2; ++i3) {
      double th3 = th3_vals[i3];
      double c3_ = COS_FN(th3), s3_ = SIN_FN(th3);
      double f1 = a3_ * c3_ - d4_ * s3_ + a2_;
      double f2 = a3_ * s3_ + d4_ * c3_;
      double f3 = d3_;

      for (int j2 = 0; j2 < 2; ++j2) {
        if (!th2_valid[i3][j2]) {
          continue;
        }
        double th2 = th2_vals[i3][j2];
        double c2_ = COS_FN(th2), s2_ = SIN_FN(th2);

        double g1 = f1 * c2_ - f2 * s2_;
        double g2 = -f3;

        double th1 = ATAN2_FN(y, x) - ATAN2_FN(g2, g1);
        // 归一化到 [-π, π]
        if (th1 > M_PI) {
          th1 -= 2.0 * M_PI;
        }
        if (th1 < -M_PI) {
          th1 += 2.0 * M_PI;
        }

        // 计算 R03 = R01 * R12 * R23
        double c1_ = COS_FN(th1), s1_ = SIN_FN(th1);

        Eigen::Matrix3d R01, R12, R23;
        R01 << c1_, -s1_, 0, s1_, c1_, 0, 0, 0, 1;
        R12 << c2_, -s2_, 0, 0, 0, -1, s2_, c2_, 0;
        R23 << c3_, -s3_, 0, s3_, c3_, 0, 0, 0, 1;

        // R34ORG = Rx(-90°) — T34 中 theta4 为零时的旋转部分
        Eigen::Matrix3d R34ORG;
        R34ORG << 1, 0, 0, 0, 0, 1, 0, -1, 0;

        Eigen::Matrix3d R04ORG = R01 * R12 * R23 * R34ORG;
        Eigen::Matrix3d R4ORG6 = R04ORG.transpose() * R06;

        // 从 R4ORG6 = Rz(th4)*Ry(-th5)*Rz(th6) 中提取腕部关节角（类 ZYZ）
        double r11 = R4ORG6(0, 0), r12 = R4ORG6(0, 1), r13 = R4ORG6(0, 2);
        double r23 = R4ORG6(1, 2);
        double r31 = R4ORG6(2, 0), r32 = R4ORG6(2, 1), r33 = R4ORG6(2, 2);

        double th4_arr[2], th5_arr[2], th6_arr[2];

        if (abs(r31) < kEps && abs(r32) < kEps) {
          // 奇异情况：th5 = 0 或 π
          th5_arr[0] = 0.0;
          th4_arr[0] = 0.0;
          th6_arr[0] = ATAN2_FN(-r12, r11);

          th5_arr[1] = M_PI;
          th4_arr[1] = 0.0;
          th6_arr[1] = ATAN2_FN(r12, -r11);
        } else {
          double s5_1 = sqrt(r31 * r31 + r32 * r32);

          th5_arr[0] = -ATAN2_FN(s5_1, r33);
          double inv_s5 = 1.0 / SIN_FN(-th5_arr[0]);
          th4_arr[0] = ATAN2_FN(r23 * inv_s5, r13 * inv_s5);
          th6_arr[0] = ATAN2_FN(r32 * inv_s5, -r31 * inv_s5);

          th5_arr[1] = -ATAN2_FN(-s5_1, r33);
          inv_s5 = 1.0 / SIN_FN(-th5_arr[1]);
          th4_arr[1] = ATAN2_FN(r23 * inv_s5, r13 * inv_s5);
          th6_arr[1] = ATAN2_FN(r32 * inv_s5, -r31 * inv_s5);
        }

        for (int kw = 0; kw < 2; ++kw) {
          Eigen::Matrix<double, 6, 1> theta;
          theta << th1, th2, th3, th4_arr[kw], th5_arr[kw], th6_arr[kw];
          solutions.push_back(theta);
        }
      }
    }

    // =======================================================================
    // 阶段 3：后处理管线 — 去重 → 肘部筛选 → URDF 转换
    //   去重后通常 ≤ 8 组，肘部筛选后理想剩余 2 组 (正手 / 反手)
    // =======================================================================

    // 3a. 去除重复解（容差范围内）
    const double dup_eps = 1e-4;
    std::vector<Eigen::Matrix<double, 6, 1>> unique;
    for (const auto &s : solutions) {
      bool is_dup = false;
      for (const auto &u : unique) {
        if ((s - u).norm() < dup_eps) {
          is_dup = true;
          break;
        }
      }
      if (!is_dup) {
        unique.push_back(s);
      }
    }

    // 3b. 筛选肘部朝上的构型（模型 theta），理想情况下剩余 2 解（正反手）
    unique = FilterElbowUpSolutions(unique);

    // 3c. 模型 theta → URDF 转换后返回
    for (auto &s : unique) {
      s = modelToUrdf(s);
    }
    return unique;
  }

  /// DH 参数访问器
  double a2() const { return a2_; }
  double a3() const { return a3_; }
  double d3() const { return d3_; }
  double d4() const { return d4_; }

  // ---------------------------------------------------------------------------
  // URDF ↔ 模型关节转换
  //
  // URDF 中 j3 的零点与 MDH 模型中 th3=0 相差 +π/2。
  //   model_th3 = wrapToPi(urdf_q3 - π/2)
  //   urdf_q3   = wrapToPi(model_th3 + π/2)
  // ---------------------------------------------------------------------------

  /// 将弧度角归一化到 [-π, π] 区间。
  static double wrapToPi(double rad) {
    rad = fmod(rad + M_PI, 2.0 * M_PI);
    if (rad < 0.0) {
      rad += 2.0 * M_PI;
    }
    return rad - M_PI;
  }

  /// 将 6 个 URDF 关节角转换为模型 theta（内部 MDH 约定）。
  static Eigen::Matrix<double, 6, 1> urdfToModel(const Eigen::Matrix<double, 6, 1> &q) {
    Eigen::Matrix<double, 6, 1> th = q;
    th(2) = wrapToPi(th(2) - M_PI_2);
    return th;
  }

  /// 将 6 个模型 theta（内部 MDH）转换为 URDF 关节角。
  static Eigen::Matrix<double, 6, 1> modelToUrdf(const Eigen::Matrix<double, 6, 1> &th) {
    Eigen::Matrix<double, 6, 1> q = th;
    q(2) = wrapToPi(q(2) + M_PI_2);
    return q;
  }

  // ---------------------------------------------------------------------------
  // 解筛选
  // ---------------------------------------------------------------------------

  /// 仅保留肘部朝上的解（th2 > 0, th3 < 0），理想情况保留 2 组（正反手）。
  static std::vector<Eigen::Matrix<double, 6, 1>> FilterElbowUpSolutions(
      const std::vector<Eigen::Matrix<double, 6, 1>> &solutions) {
    std::vector<Eigen::Matrix<double, 6, 1>> filtered;
    for (const auto &s : solutions) {
      const auto urdf_q = modelToUrdf(s);
      if (urdf_q(1) > 0.0 && urdf_q(2) < 0.0) {  // 肘部朝上，腕部朝下
        filtered.push_back(s);
      }
    }
    return filtered;
  }

 private:
  double a2_, a3_, d3_, d4_;
  double K_;             // a2^2 + a3^2 + d3^2 + d4^2
  double b_coeff_;       // 4 * a2 * d4
  double offset_2a2a3_;  // 2 * a2 * a3
};

}  // namespace amorph_kinematics