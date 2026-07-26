#pragma once

#include <Eigen/Dense>
// #include <arm_math.h>

#define SIN_FN std::sin
#define COS_FN std::cos
// #define SIN_FN arm_sin_f32
// #define COS_FN arm_cos_f32

class Kinematics {
  /**
   * @brief 角度平滑处理（最短路径原则）
   * 将原始目标角度 target_angle 转换到距离 current_angle 最近的等价角度
   * 防止出现 179度 突变到 -179度 导致电机转一整圈的问题
   */
  static float OptimizeAngle(float target_angle, float current_angle) {
    float delta = target_angle - current_angle;
    // 将差值映射到 [-M_PI, M_PI] 范围内
    delta = std::fmod(delta + M_PI, 2.0f * M_PI);
    if (delta < 0.0f) {
      delta += 2.0f * M_PI;
    }
    delta -= M_PI;
    // 在当前角度的基础上，加上最短的旋转差值
    return current_angle + delta;
  }

 public:
  // 机械臂几何参数 (单位: mm)
  struct {
    float l1 = 0.40883f;
    float l2 = 0.49f;
    float dz = 0.04f;
  } params_;

  Kinematics() = default;

  std::optional<std::array<float, 6>> SolveIk(const Eigen::Vector3f& target_position,
                                              const Eigen::Matrix3f& target_rotation,
                                              const std::array<float, 6>& current_joints) {
    const double x = static_cast<double>(target_position.x());
    const double y = static_cast<double>(target_position.y());
    const double z = static_cast<double>(target_position.z());
    const Eigen::Matrix3d Tgt_R = target_rotation.cast<double>();

    // 位置解
    const double a1 = std::atan2(y, x);
    const double r_val = std::sqrt(x * x + y * y);
    const double z_eff = z - static_cast<double>(params_.dz);
    const double L_sq = r_val * r_val + z_eff * z_eff;

    const double cos_t3 = (L_sq - std::pow(params_.l1, 2) - std::pow(params_.l2, 2)) / (2.0 * params_.l1 * params_.l2);
    if (std::abs(cos_t3) > 1.0) {
      return std::nullopt;
    }

    const double a3 = -std::acos(cos_t3);  // Elbow Up

    const double k1 = params_.l1 + params_.l2 * std::cos(a3);
    const double k2 = params_.l2 * std::sin(a3);
    const double a2 = std::atan2(z_eff, r_val) - std::atan2(k2, k1);

    // 姿态解
    const Eigen::Matrix3d R_z1 = Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const double a23 = -(a2 + a3);
    const Eigen::Matrix3d R_y23 = Eigen::AngleAxisd(a23, Eigen::Vector3d::UnitY()).toRotationMatrix();

    const Eigen::Matrix3d R03 = R_z1 * R_y23;
    const Eigen::Matrix3d W = R03.transpose() * Tgt_R;

    // 后三轴解
    const double c5 = std::clamp(W(0, 0), -1.0, 1.0);
    const double a5 = std::acos(c5);

    double a4, a6;
    if (std::abs(std::sin(a5)) < 1e-7) {
      a4 = 0.0;
      a6 = std::atan2(W(2, 1), W(1, 1));
    } else {
      a4 = std::atan2(W(1, 0), -W(2, 0));
      a6 = std::atan2(W(0, 1), W(0, 2));
    }

    const std::array<float, 6> raw_joints = {static_cast<float>(a1),  static_cast<float>(a2),  static_cast<float>(a3),
                                             static_cast<float>(-a4), static_cast<float>(-a5), static_cast<float>(-a6)};

    // 就近原则优化，防止多解导致角度跳变
    std::array<float, 6> optimized_joints;
    for (int i = 0; i < 6; ++i) {
      optimized_joints[i] = OptimizeAngle(raw_joints[i], current_joints[i]);
    }

    return optimized_joints;
  }

  /**
   * @brief 6-DOF 正向运动学 (Forward Kinematics)
   * @param[in] angles 包含6个关节角的数组 [t1, t2, t3, t4, t5, t6] (弧度)
   * @param[out] out_position 末端位置
   * @param[out] out_rotation 末端旋转矩阵
   */
  void SolveFk(const std::array<float, 6>& angles, Eigen::Vector3f& out_position, Eigen::Matrix3f& out_rotation) const {
    const float t1 = angles[0], t2 = angles[1], t3 = angles[2];
    const float t4 = angles[3], t5 = angles[4], t6 = angles[5];

    // 位置部分 (基于第一、二段连杆)
    const float r2 = params_.l1 * COS_FN(t2);
    const float z2 = params_.dz + params_.l1 * SIN_FN(t2);

    const float r3 = r2 + params_.l2 * COS_FN(t2 + t3);
    const float z3 = z2 + params_.l2 * SIN_FN(t2 + t3);

    out_position.x() = r3 * COS_FN(t1);
    out_position.y() = r3 * SIN_FN(t1);
    out_position.z() = z3;

    // 旋转部分：第一阶段 (R03)
    const float a23 = -(t2 + t3);
    const float s_t1 = SIN_FN(t1), c_t1 = COS_FN(t1);
    const float s_a23 = SIN_FN(a23), c_a23 = COS_FN(a23);

    // clang-format off
    Eigen::Matrix3f R03;
    R03 << c_t1 * c_a23, -s_t1,  c_t1 * s_a23,
           s_t1 * c_a23,  c_t1,  s_t1 * s_a23,
           -s_a23,        0.0,   c_a23;
    //clang-format on

    // 旋转部分：手腕 (R_wrist: R_x(t4) * R_y(t5) * R_x(t6))
    const float s4 = SIN_FN(t4), c4 = COS_FN(t4);
    const float s5 = SIN_FN(t5), c5 = COS_FN(t5);
    const float s6 = SIN_FN(t6), c6 = COS_FN(t6);

    // clang-format off
    Eigen::Matrix3f R_x4;
    R_x4 << 1.0, 0.0, 0.0,
            0.0, c4,  -s4,
            0.0, s4,  c4;

    Eigen::Matrix3f R_y5;
    R_y5 << c5,  0.0, s5,
            0.0, 1.0, 0.0,
            -s5, 0.0, c5;

    Eigen::Matrix3f R_x6;
    R_x6 << 1.0, 0.0, 0.0,
            0.0, c6,  -s6,
            0.0, s6,  c6;
    // clang-format on

    out_rotation = R03 * R_x4 * R_y5 * R_x6;
  }
};
