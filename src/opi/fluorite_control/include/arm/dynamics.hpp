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

namespace fluorite::arm {

/**
 * @brief   机械臂的动力学模型，用于在pitch1、pitch2两个关节上叠加重力补偿，改善PID控制器的性能
 * @details 建模和仿真见 仓库根目录/MATLAB/dynamic_model.m 和 仓库根目录/MATLAB/ss_model_validate.slx
 */
class DynamicModel {
 public:
  using Vec3 = Eigen::Vector3d;
  using Mat3x3 = Eigen::Matrix3d;
  Vec3 operator()(const Vec3& q, const Vec3& qd, bool logging = false);

  /**
   * @brief 计算逆动力学,q的定义见 仓库根目录/assets/symdef.png
   * @param q       关节角度
   * @param qd      关节角速度
   * @param qdd     关节加速度
   * @param logging 是否打印日志
   * @return Vec3   关节力矩
   */
  Vec3 operator()(const Vec3& q, const Vec3& qd, const Vec3& qdd, bool logging = false);

 private:
  std::pair<rclcpp::Time, Vec3> last_qd_{};
  Mat3x3 m_{}, n_{};
  Vec3 g_{};

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