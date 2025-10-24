
#pragma once

#include <Eigen/Dense>

#include "arm_math.h"

/**
 * @brief   机械臂的动力学模型，可以用这个类计算正动力学，或者转换成状态空间形式的模型
 */
class DynamicModel {
 public:
  using Vec3 = Eigen::Vector3f;
  using Mat3x3 = Eigen::Matrix3f;
  DynamicModel() {
    // 初始化CMSIS-DSP矩阵结构
    arm_mat_init_f32(&m_cmsis_, 3, 3, m_data_);
    arm_mat_init_f32(&n_cmsis_, 3, 3, n_data_);
    arm_mat_init_f32(&g_cmsis_, 3, 1, g_data_);
    arm_mat_init_f32(&qdd_cmsis_, 3, 1, qdd_data_);
    arm_mat_init_f32(&qd_cmsis_, 3, 1, qd_data_);
    arm_mat_init_f32(&temp_vec_cmsis_, 3, 1, temp_vec_data_);
    arm_mat_init_f32(&temp_mat_cmsis_, 3, 3, temp_mat_data_);
    arm_mat_init_f32(&m_inv_cmsis_, 3, 3, m_inv_data_);
  }

  void Relinearize(const Vec3& q, const Vec3& qd) {
    const float q2 = q(1);
    const float q3 = q(2);
    const float dq1 = qd(0);
    const float dq2 = qd(1);
    const float dq3 = qd(2);

    // 使用CMSIS-DSP三角函数
    const float cos_q2 = arm_cos_f32(q2);
    const float sin_q2 = arm_sin_f32(q2);
    const float cos_q3 = arm_cos_f32(q3);
    const float sin_q3 = arm_sin_f32(q3);
    const float cos_q2_q3 = arm_cos_f32(q2 + q3);
    const float sin_q2_q3 = arm_sin_f32(q2 + q3);
    const float cos_2q2 = arm_cos_f32(2 * q2);
    const float sin_2q2 = arm_sin_f32(2 * q2);
    const float cos_2q2_2q3 = arm_cos_f32(2 * q2 + 2 * q3);
    const float sin_2q2_2q3 = arm_sin_f32(2 * q2 + 2 * q3);
    const float cos_2q2_q3 = arm_cos_f32(2 * q2 + q3);
    const float sin_2q2_q3 = arm_sin_f32(2 * q2 + q3);

    // 计算质量矩阵 m_
    m_data_[0] = i1 + a1 * cos_q2 * cos_q2 + a2 * cos_q2_q3 * cos_q2_q3 + 2 * a3 * cos_q2 * cos_q2_q3;
    m_data_[1] = 0.0f;
    m_data_[2] = 0.0f;
    m_data_[3] = 0.0f;
    m_data_[4] = i2 + a1 + a2 + 2 * a3 * cos_q3;
    m_data_[5] = a2 + a3 * cos_q3;
    m_data_[6] = 0.0f;
    m_data_[7] = a2 + a3 * cos_q3;
    m_data_[8] = i3 + a2;

    // 计算科里奥利矩阵 n_
    n_data_[0] = -0.5f * a1 * dq2 * sin_2q2 - 0.5f * a2 * (dq2 + dq3) * sin_2q2_2q3 - 
                  a3 * dq2 * sin_2q2_q3 - a3 * dq3 * cos_q2 * sin_q2_q3;
    n_data_[1] = -0.5f * a1 * dq1 * sin_2q2 - 0.5f * a2 * dq1 * sin_2q2_2q3 - a3 * dq1 * sin_2q2_q3;
    n_data_[2] = -0.5f * a1 * dq1 * sin_2q2_2q3 - a3 * dq1 * cos_q2 * sin_q2_q3;
    n_data_[3] = 0.5f * a1 * dq1 * sin_2q2 + 0.5f * a2 * dq1 * sin_2q2_2q3 + a3 * dq1 * sin_2q2_q3;
    n_data_[4] = -a3 * dq3 * sin_q3;
    n_data_[5] = -a3 * (dq2 + dq3) * sin_q3;
    n_data_[6] = 0.5f * a1 * dq1 * sin_2q2_2q3 + a3 * dq1 * cos_q2 * sin_q2_q3;
    n_data_[7] = -a3 * dq2 * sin_q3;
    n_data_[8] = 0.0f;

    // 计算重力向量 g_
    g_data_[0] = 0.0f;
    g_data_[1] = b1 * cos_q2 + b2 * cos_q2_q3;
    g_data_[2] = b2 * cos_q2_q3;

    // 更新Eigen矩阵以保持API兼容性
    m_ = Eigen::Map<Mat3x3>(m_data_);
    n_ = Eigen::Map<Mat3x3>(n_data_);
    g_ = Eigen::Map<Vec3>(g_data_);
  }

  Vec3 ForwardDynamics(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool relinearize = true) {
    if (relinearize) {
      Relinearize(q, qd);
    }

    // 使用CMSIS-DSP进行矩阵运算
    // 计算 m_ * qdd
    qdd_data_[0] = qdd(0);
    qdd_data_[1] = qdd(1);
    qdd_data_[2] = qdd(2);
    arm_mat_mult_f32(&m_cmsis_, &qdd_cmsis_, &temp_vec_cmsis_);

    // 计算 n_ * qd
    qd_data_[0] = qd(0);
    qd_data_[1] = qd(1);
    qd_data_[2] = qd(2);
    arm_mat_mult_f32(&n_cmsis_, &qd_cmsis_, &temp_vec_cmsis_);

    // 计算 m_ * qdd + n_ * qd + g_
    float result_data[3];
    arm_matrix_instance_f32 result_cmsis;
    arm_mat_init_f32(&result_cmsis, 3, 1, result_data);
    
    // 先计算 m_ * qdd + n_ * qd
    arm_mat_add_f32(&temp_vec_cmsis_, &temp_vec_cmsis_, &result_cmsis);
    // 再加上 g_
    arm_mat_add_f32(&result_cmsis, &g_cmsis_, &result_cmsis);

    return Eigen::Map<Vec3>(result_data);
  }

  const auto& StateSpaceModel() {
    const Mat3x3 zeros = Mat3x3::Zero();
    const Mat3x3 eye = Mat3x3::Identity();

    // 使用CMSIS-DSP计算 m_ 的逆矩阵
    arm_mat_inverse_f32(&m_cmsis_, &m_inv_cmsis_);
    
    // 计算 -m_.inverse() * n_
    arm_mat_mult_f32(&m_inv_cmsis_, &n_cmsis_, &temp_mat_cmsis_);
    arm_mat_scale_f32(&temp_mat_cmsis_, -1.0f, &temp_mat_cmsis_);

    // 构建状态空间模型
    ss_model_.A.setZero();
    // 左上角3x3为零矩阵
    // 右上角3x3为负单位矩阵
    ss_model_.A.block<3,3>(0,3) = -eye;
    // 左下角3x3为零矩阵
    // 右下角3x3为 -m_.inverse() * n_
    ss_model_.A.block<3,3>(3,3) = Eigen::Map<Mat3x3>(temp_mat_data_);

    ss_model_.B.setZero();
    ss_model_.B.block<3,3>(3,0) = eye;
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

  // CMSIS-DSP矩阵实例和数据存储
  arm_matrix_instance_f32 m_cmsis_;
  arm_matrix_instance_f32 n_cmsis_;
  arm_matrix_instance_f32 g_cmsis_;
  arm_matrix_instance_f32 qdd_cmsis_;
  arm_matrix_instance_f32 qd_cmsis_;
  arm_matrix_instance_f32 temp_vec_cmsis_;
  arm_matrix_instance_f32 temp_mat_cmsis_;
  arm_matrix_instance_f32 m_inv_cmsis_;
  
  float m_data_[9];
  float n_data_[9];
  float g_data_[3];
  float qdd_data_[3];
  float qd_data_[3];
  float temp_vec_data_[3];
  float temp_mat_data_[9];
  float m_inv_data_[9];

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
