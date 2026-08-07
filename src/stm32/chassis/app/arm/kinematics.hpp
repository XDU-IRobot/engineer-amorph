#pragma once

#include <Eigen/Dense>
#include <etl/vector.h>

#include <optional>

#include "bsp/cmsis_dsp_helper.hpp"

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
  ArmKinematics(float a2 = 0.472254f, float a3 = 0.f, float d3 = 0.f, float d4 = 0.45)
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
  Eigen::Matrix4f SolveFk(const Eigen::Matrix<float, 6, 1> &theta) const {
    // 将 URDF 关节角转换为内部 MDH 约定
    const Eigen::Matrix<float, 6, 1> th = urdfToModel(theta);

    // ---- sin/cos（CMSIS-DSP）----------------------------------------------
    const float c1 = COS_FN(th(0)), s1 = SIN_FN(th(0));
    const float c2 = COS_FN(th(1)), s2 = SIN_FN(th(1));
    const float c3 = COS_FN(th(2)), s3 = SIN_FN(th(2));
    const float c4 = COS_FN(th(3)), s4 = SIN_FN(th(3));
    const float c5 = COS_FN(th(4)), s5 = SIN_FN(th(4));
    const float c6 = COS_FN(th(5)), s6 = SIN_FN(th(5));

    // ---- 旋转矩阵数据（CMSIS-DSP 行主序）----------------------------------
    // clang-format off
    float R01_d[9] = {  c1, -s1,  0,   s1,  c1,  0,    0,   0,  1 };
    float R12_d[9] = {  c2, -s2,  0,    0,   0, -1,   s2,  c2,  0 };
    float R23_d[9] = {  c3, -s3,  0,   s3,  c3,  0,    0,   0,  1 };
    float R34_d[9] = {  c4, -s4,  0,    0,   0,  1,  -s4, -c4,  0 };
    float R45_d[9] = {  c5, -s5,  0,    0,   0, -1,   s5,  c5,  0 };
    float R56_d[9] = {  c6, -s6,  0,    0,   0,  1,  -s6, -c6,  0 };
    // clang-format on

    arm_matrix_instance_f32 R01, R12, R23, R34, R45, R56;
    arm_mat_init_f32(&R01, 3, 3, R01_d);
    arm_mat_init_f32(&R12, 3, 3, R12_d);
    arm_mat_init_f32(&R23, 3, 3, R23_d);
    arm_mat_init_f32(&R34, 3, 3, R34_d);
    arm_mat_init_f32(&R45, 3, 3, R45_d);
    arm_mat_init_f32(&R56, 3, 3, R56_d);

    // ---- 平移向量 -----------------------------------------------------------
    // p23 = [a2; 0; d3], p34 = [a3; d4; 0]
    const float p23_d[3] = {a2_, 0.0f, d3_};
    const float p34_d[3] = {a3_, d4_, 0.0f};
    arm_matrix_instance_f32 p23, p34;
    arm_mat_init_f32(&p23, 3, 1, (float *)p23_d);
    arm_mat_init_f32(&p34, 3, 1, (float *)p34_d);

    // ---- p24 = p23 + R23 * p34（末端在坐标系2中的位置）--------------------
    float tmp[3];
    arm_matrix_instance_f32 tmp_vec;
    arm_mat_init_f32(&tmp_vec, 3, 1, tmp);
    arm_mat_mult_f32(&R23, &p34, &tmp_vec);  // tmp = R23 * p34
    const float p24_d[3] = {p23_d[0] + tmp[0], p23_d[1] + tmp[1], p23_d[2] + tmp[2]};

    arm_matrix_instance_f32 p24;
    arm_mat_init_f32(&p24, 3, 1, (float *)p24_d);

    // ---- p = R01 * R12 * p24（末端在基坐标系中的位置）---------------------
    arm_mat_mult_f32(&R12, &p24, &tmp_vec);  // tmp = R12 * p24
    float p_d[3];
    arm_matrix_instance_f32 p_mat;
    arm_mat_init_f32(&p_mat, 3, 1, p_d);
    arm_mat_mult_f32(&R01, &tmp_vec, &p_mat);  // p = R01 * tmp

    // ---- R = R01 * R12 * R23 * R34 * R45 * R56（末端姿态）-----------------
    float M1[9], M2[9], M3[9], M4[9];
    arm_matrix_instance_f32 m1, m2, m3, m4;
    arm_mat_init_f32(&m1, 3, 3, M1);
    arm_mat_init_f32(&m2, 3, 3, M2);
    arm_mat_init_f32(&m3, 3, 3, M3);
    arm_mat_init_f32(&m4, 3, 3, M4);

    arm_mat_mult_f32(&R01, &R12, &m1);  // M1 = R01 * R12
    arm_mat_mult_f32(&m1, &R23, &m2);   // M2 = R01*R12 * R23
    arm_mat_mult_f32(&m2, &R34, &m3);   // M3 = ... * R34
    arm_mat_mult_f32(&m3, &R45, &m4);   // M4 = ... * R45

    float R_d[9];
    arm_matrix_instance_f32 R_mat;
    arm_mat_init_f32(&R_mat, 3, 3, R_d);
    arm_mat_mult_f32(&m4, &R56, &R_mat);  // R = ... * R56

    // ---- 构建 4x4 齐次变换矩阵（拷贝回 Eigen）-----------------------------
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T(r, c) = R_d[r * 3 + c];
      }
    }
    T(0, 3) = p_d[0];
    T(1, 3) = p_d[1];
    T(2, 3) = p_d[2];
    return T;
  }

  // ---------------------------------------------------------------------------
  // 逆运动学
  // ---------------------------------------------------------------------------

  /**
   * @brief 对期望末端位姿求解逆运动学。
   *
   * 使用 PUMA560 构型机械臂的解析闭式解。
   * 内部自动完成去重、肘部朝上筛选，并从剩余解中选取与当前关节角最接近者。
   *
   * @param T_target 期望的 4x4 齐次变换矩阵 (T0_6)。
   * @param current_q 当前关节角（URDF 约定），用于在多解中选最接近者。
   * @return 有效解时返回 6x1 关节角（URDF 约定），无解时返回 std::nullopt。
   */
  std::optional<Eigen::Matrix<float, 6, 1>> SolveIk(const Eigen::Matrix4f &T_target,
                                                    const Eigen::Matrix<float, 6, 1> &current_q) const {
    // =======================================================================
    // 阶段 1：解析求解前三个关节角 (th3 → th2 → th1)
    // th3: 2 种取值 (肘部上下)
    // th2: 每 th3 最多 2 种 (± 分支)，取决于 a2_eq != 0
    // th1: 由 atan2 直接计算
    // =======================================================================

    constexpr float kEps = 1e-12;

    // 从 T_target 提取位置和姿态（Eigen 列主序 → CMSIS-DSP 行主序）
    float x = T_target(0, 3), y = T_target(1, 3), z = T_target(2, 3);
    float r2 = x * x + y * y + z * z;
    float R06_d[9] = {
        T_target(0, 0), T_target(0, 1), T_target(0, 2),
        T_target(1, 0), T_target(1, 1), T_target(1, 2),
        T_target(2, 0), T_target(2, 1), T_target(2, 2),
    };
    arm_matrix_instance_f32 R06;
    arm_mat_init_f32(&R06, 3, 3, R06_d);

    etl::vector<Eigen::Matrix<float, 6, 1>, 8> solutions;
    solutions.reserve(8);

    // ---- 求解 theta3 -------------------------------------------------------
    float a = r2 - K_ + offset_2a2a3_;
    float c = r2 - K_ - offset_2a2a3_;
    float Del3 = b_coeff_ * b_coeff_ - 4.0 * a * c;

    if (Del3 < 0.0) {
      return std::nullopt;  // 无解
    }

    float sqrt_Del3 = SQRT_FN(Del3);
    float t3_1 = (-b_coeff_ + sqrt_Del3) / (2.0 * a);
    float t3_2 = (-b_coeff_ - sqrt_Del3) / (2.0 * a);

    // th3 有 2 个可能取值，每个产生 2 个 theta2 取值
    float th3_vals[2] = {2.0f * atan(t3_1), 2.0f * atan(t3_2)};

    // ---- 对每个 theta3 求解 theta2 -----------------------------------------
    // 结果：th2 矩阵 [2 x 2]，行 = theta3 下标，列 = +/- 分支
    float th2_vals[2][2];
    bool th2_valid[2][2] = {{false, false}, {false, false}};

    for (int i3 = 0; i3 < 2; ++i3) {
      float th3 = th3_vals[i3];
      float c3 = COS_FN(th3), s3 = SIN_FN(th3);

      float f1 = a3_ * c3 - d4_ * s3 + a2_;
      float f2 = a3_ * s3 + d4_ * c3;

      float Del2 = 4.0 * f1 * f1 - 4.0 * (z * z - f2 * f2);
      if (Del2 < 0.0) {
        continue;
      }
      float sqrt_Del2 = SQRT_FN(Del2);

      float a2_eq = z + f2;
      float b2_eq = -2.0 * f1;
      (void)(z - f2);  // c2_eq，仅在公式推导中引用

      if (abs(a2_eq) > kEps) {
        float t2_1 = (-b2_eq + sqrt_Del2) / (2.0 * a2_eq);
        float t2_2 = (-b2_eq - sqrt_Del2) / (2.0 * a2_eq);
        th2_vals[i3][0] = 2.0 * atan(t2_1);
        th2_vals[i3][1] = 2.0 * atan(t2_2);
        th2_valid[i3][0] = true;
        th2_valid[i3][1] = true;
      }
    }

    // ---- 预初始化常量矩阵 R34ORG = Rx(-90°)（行主序）------------------------
    const float R34ORG_d[9] = {1, 0, 0, 0, 0, 1, 0, -1, 0};
    arm_matrix_instance_f32 R34ORG;
    arm_mat_init_f32(&R34ORG, 3, 3, (float *)R34ORG_d);

    // ---- 阶段 2 复用缓冲区 --------------------------------------------------
    float M1_d[9], M2_d[9], M3_d[9], M3T_d[9], R4ORG6_d[9];
    arm_matrix_instance_f32 M1, M2, M3, M3T, R4ORG6;
    arm_mat_init_f32(&M1, 3, 3, M1_d);
    arm_mat_init_f32(&M2, 3, 3, M2_d);
    arm_mat_init_f32(&M3, 3, 3, M3_d);
    arm_mat_init_f32(&M3T, 3, 3, M3T_d);
    arm_mat_init_f32(&R4ORG6, 3, 3, R4ORG6_d);

    // =======================================================================
    // 阶段 2：对每组 (th3, th2) 组合求解 th1 和腕部角度 (th4/th5/th6)
    // 腕部每组合有 2 种解（正手/反手），因此最多产生 2×2×2 = 8 组解
    // =======================================================================
    for (int i3 = 0; i3 < 2; ++i3) {
      float th3 = th3_vals[i3];
      float c3_ = COS_FN(th3), s3_ = SIN_FN(th3);
      float f1 = a3_ * c3_ - d4_ * s3_ + a2_;
      float f2 = a3_ * s3_ + d4_ * c3_;
      float f3 = d3_;

      for (int j2 = 0; j2 < 2; ++j2) {
        if (!th2_valid[i3][j2]) {
          continue;
        }
        float th2 = th2_vals[i3][j2];
        float c2_ = COS_FN(th2), s2_ = SIN_FN(th2);

        float g1 = f1 * c2_ - f2 * s2_;
        float g2 = -f3;

        float th1 = ATAN2_FN(y, x) - ATAN2_FN(g2, g1);
        // 归一化到 [-π, π]
        if (th1 > M_PI) {
          th1 -= 2.0 * M_PI;
        }
        if (th1 < -M_PI) {
          th1 += 2.0 * M_PI;
        }

        float c1_ = COS_FN(th1), s1_ = SIN_FN(th1);

        // ---- CMSIS-DSP 旋转矩阵（行主序）------------------------------------
        float R01_d[9] = {c1_, -s1_, 0, s1_, c1_, 0, 0, 0, 1};
        float R12_d[9] = {c2_, -s2_, 0, 0, 0, -1, s2_, c2_, 0};
        float R23_d[9] = {c3_, -s3_, 0, s3_, c3_, 0, 0, 0, 1};
        arm_matrix_instance_f32 R01, R12, R23;
        arm_mat_init_f32(&R01, 3, 3, R01_d);
        arm_mat_init_f32(&R12, 3, 3, R12_d);
        arm_mat_init_f32(&R23, 3, 3, R23_d);

        // R04ORG = R01 * R12 * R23 * R34ORG
        arm_mat_mult_f32(&R01, &R12, &M1);     // M1 = R01 * R12
        arm_mat_mult_f32(&M1, &R23, &M2);      // M2 = R01*R12 * R23
        arm_mat_mult_f32(&M2, &R34ORG, &M3);   // M3 = R04ORG

        // R4ORG6 = R04ORG^T * R06
        arm_mat_trans_f32(&M3, &M3T);          // M3T = R04ORG^T
        arm_mat_mult_f32(&M3T, &R06, &R4ORG6); // R4ORG6 = R04ORG^T * R06

        // 从 R4ORG6（行主序，r*3 + c）提取腕部关节角元素
        float r11 = R4ORG6_d[0], r12 = R4ORG6_d[1], r13 = R4ORG6_d[2];
        float r23 = R4ORG6_d[5];   // row 1, col 2
        float r31 = R4ORG6_d[6], r32 = R4ORG6_d[7], r33 = R4ORG6_d[8];

        float th4_arr[2], th5_arr[2], th6_arr[2];

        if (abs(r31) < kEps && abs(r32) < kEps) {
          // 奇异情况：th5 = 0 或 π
          th5_arr[0] = 0.0;
          th4_arr[0] = 0.0;
          th6_arr[0] = ATAN2_FN(-r12, r11);

          th5_arr[1] = M_PI;
          th4_arr[1] = 0.0;
          th6_arr[1] = ATAN2_FN(r12, -r11);
        } else {
          float s5_1 = SQRT_FN(r31 * r31 + r32 * r32);

          th5_arr[0] = -ATAN2_FN(s5_1, r33);
          float inv_s5 = 1.0 / SIN_FN(-th5_arr[0]);
          th4_arr[0] = ATAN2_FN(r23 * inv_s5, r13 * inv_s5);
          th6_arr[0] = ATAN2_FN(r32 * inv_s5, -r31 * inv_s5);

          th5_arr[1] = -ATAN2_FN(-s5_1, r33);
          inv_s5 = 1.0 / SIN_FN(-th5_arr[1]);
          th4_arr[1] = ATAN2_FN(r23 * inv_s5, r13 * inv_s5);
          th6_arr[1] = ATAN2_FN(r32 * inv_s5, -r31 * inv_s5);
        }

        for (int kw = 0; kw < 2; ++kw) {
          Eigen::Matrix<float, 6, 1> theta;
          theta << th1, th2, th3, th4_arr[kw], th5_arr[kw], th6_arr[kw];
          solutions.push_back(theta);
        }
      }
    }

    // =======================================================================
    // 阶段 3：后处理管线 — 去重 → 肘部筛选 + 最近解选取 → URDF 转换
    //   去重后通常 ≤ 8 组，肘部筛选后理想剩余 2~4 组，最终选 1 组
    // =======================================================================

    // 3a. 去除重复解（容差范围内，手动 L2 范数）
    const float dup_eps = 1e-4;
    etl::vector<Eigen::Matrix<float, 6, 1>, 8> unique;
    for (const auto &s : solutions) {
      bool is_dup = false;
      for (const auto &u : unique) {
        float sum_sq = 0;
        for (int k = 0; k < 6; ++k) {
          float d = s(k) - u(k);
          sum_sq += d * d;
        }
        if (SQRT_FN(sum_sq) < dup_eps) {
          is_dup = true;
          break;
        }
      }
      if (!is_dup) {
        unique.push_back(s);
      }
    }

    if (unique.empty()) {
      return std::nullopt;
    }

    // 3b. 筛选肘部朝上构型，并选取与当前关节角最接近的解（模型 theta 空间比较）
    const auto best_model_opt = FilterElbowUpSolutions(unique, urdfToModel(current_q));
    if (!best_model_opt.has_value()) {
      return std::nullopt;  // 无肘部朝上解
    }

    // 3c. 模型 theta → URDF 转换
    Eigen::Matrix<float, 6, 1> result = modelToUrdf(best_model_opt.value());

    // 3d. 将结果展开到与当前关节角最接近的 2π 周期
    //     解决 j1 等跨 ±π 边界关节的跳变问题（如当前 200°,IK 返回 -160°）
    for (int i = 0; i < 6; ++i) {
      float diff = current_q(i) - result(i);
      result(i) += 2.0f * M_PI * std::round(diff / (2.0f * M_PI));
    }
    return result;
  }

  /// DH 参数访问器
  float a2() const { return a2_; }
  float a3() const { return a3_; }
  float d3() const { return d3_; }
  float d4() const { return d4_; }

  // ---------------------------------------------------------------------------
  // URDF ↔ 模型关节转换
  //
  // URDF 中 j3 的零点与 MDH 模型中 th3=0 相差 +π/2。
  //   model_th3 = wrapToPi(urdf_q3 - π/2)
  //   urdf_q3   = wrapToPi(model_th3 + π/2)
  // ---------------------------------------------------------------------------

  /// 将弧度角归一化到 [-π, π] 区间。
  static float WrapToPi(float rad) {
    rad = fmod(rad + M_PI, 2.0 * M_PI);
    if (rad < 0.0) {
      rad += 2.0 * M_PI;
    }
    return rad - M_PI;
  }

  /// 将 6 个 URDF 关节角转换为模型 theta（内部 MDH 约定）。
  static Eigen::Matrix<float, 6, 1> urdfToModel(const Eigen::Matrix<float, 6, 1> &q) {
    Eigen::Matrix<float, 6, 1> th = q;
    th(2) = WrapToPi(th(2) - M_PI_2);
    return th;
  }

  /// 将 6 个模型 theta（内部 MDH）转换为 URDF 关节角。
  static Eigen::Matrix<float, 6, 1> modelToUrdf(const Eigen::Matrix<float, 6, 1> &th) {
    Eigen::Matrix<float, 6, 1> q = th;
    q(2) = WrapToPi(q(2) + M_PI_2);
    return q;
  }

  /// 从解集中筛选肘部朝上的构型，并选取与 current_q_model 最接近的解。
  /// @return 存在肘部朝上解时返回最优解，否则返回 std::nullopt。
  static std::optional<Eigen::Matrix<float, 6, 1>> FilterElbowUpSolutions(
      const etl::vector<Eigen::Matrix<float, 6, 1>, 8> &solutions, const Eigen::Matrix<float, 6, 1> &current_q_model) {
    // 筛选肘部朝上的解
    etl::vector<Eigen::Matrix<float, 6, 1>, 8> elbow_up;
    for (const auto &s : solutions) {
      const auto urdf_q = modelToUrdf(s);
      if (urdf_q(1) > 0.0 && urdf_q(2) < 0.0) {  // 肘部朝上，腕部朝下
        elbow_up.push_back(s);
      }
    }
    if (elbow_up.empty()) {
      return std::nullopt;
    }
    // 选取与当前关节角差距（平方 L2 范数）最小的解
    float best_dist = INFINITY;
    Eigen::Matrix<float, 6, 1> best = elbow_up[0];
    for (const auto &s : elbow_up) {
      float dist = 0;
      for (int i = 0; i < 6; ++i) {
        float d = s(i) - current_q_model(i);
        dist += d * d;
      }
      if (dist < best_dist) {
        best_dist = dist;
        best = s;
      }
    }
    return best;
  }

 private:
  float a2_, a3_, d3_, d4_;
  float K_;             // a2^2 + a3^2 + d3^2 + d4^2
  float b_coeff_;       // 4 * a2 * d4
  float offset_2a2a3_;  // 2 * a2 * a3
};
