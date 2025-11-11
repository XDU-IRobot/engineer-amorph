
#pragma once

#include <array>
#include <chrono>

#include "spline_1d_interp.hpp"
#include "rpc/typedefs.hpp"

/**
 * @brief 多关节轨迹插值器类
 *
 * 该类使用样条插值方法将离散的关节轨迹点连接成连续的轨迹。
 * 支持两种使用模式：
 * 1. 静态插值：给定起点和终点，生成完整的轨迹
 * 2. 动态插值：设置完整轨迹后，可以查询任意时间点的关节位置
 *
 * @tparam NJoints 关节数量，模板参数用于支持不同自由度的机械臂
 */
template <size_t NJoints>
class JointSpaceInterpolator {
  using Trajectory = JointTrajectory;  ///< 轨迹
  using Point = JointTrajectoryPoint;  ///< 轨迹点
  using Duration = std::chrono::duration<int, std::milli>;

 public:
  /**
   * @brief 默认构造函数
   */
  JointSpaceInterpolator() = default;

  /**
   * @brief 静态插值方法 - 在两点之间生成完整的轨迹
   *
   * 该方法使用样条插值在起点和终点之间生成平滑的轨迹。
   * 每个关节独立进行一维插值，确保轨迹的平滑性。
   *
   * @param start 轨迹起点，包含各关节的初始位置
   * @param end 轨迹终点，包含各关节的目标位置
   * @param traj_duration 轨迹总持续时间
   * @param dt 轨迹点之间的时间间隔
   * @return Trajectory 生成的完整轨迹，包含所有插值点
   *
   * @note 此方法为静态方法，不依赖于类的内部状态
   * @warning 生成的轨迹点数量由 traj_duration/dt 决定，请确保 dt 能整除 traj_duration
   */
  static Trajectory Interpolate(const Point &start, const Point &end, Duration traj_duration, Duration dt) {
    // 相对来说这个函数的功能比较独立于这个类，所以为了不影响类本身的功能，这个函数单独创建一组1d插值器
    static std::array<Spline1D<float>, NJoints> interpolators_temp;

    Trajectory traj;
    const size_t num_points =
        static_cast<size_t>(std::chrono::duration_cast<std::chrono::milliseconds>(traj_duration).count() /
                            std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
    traj.points.resize(num_points);
    for (size_t i = 0; i < NJoints; ++i) {
      const std::vector x = {0.0, static_cast<float>(num_points - 1)};
      const std::vector y = {start.positions[i], end.positions[i]};
      interpolators_temp[i].setData(x, y);
    }
    for (size_t i = 0; i < num_points; ++i) {
      traj.points[i].velocities.resize(NJoints);
      traj.points[i].positions.resize(NJoints);
      traj.points[i].time_from_start = dt * i;
      for (size_t j = 0; j < NJoints; ++j) {
        traj.points[i].positions[j] = interpolators_temp[j](i);
      }
    }
    return traj;
  }

  /**
   * @brief 设置轨迹数据
   *
   * 将离散的轨迹数据设置到插值器中，为后续的实时插值做准备。
   * 每个关节都会根据时间序列建立独立的样条插值器。
   *
   * @param traj 输入的离散轨迹数据
   *
   * @note 调用此方法后，可以使用 operator() 方法查询任意时间点的关节位置
   * @warning 轨迹点的时间必须单调递增，且时间间隔应合理
   */
  void SetTrajectory(const Trajectory &traj) {
    std::vector<float> x(traj.points.size());
    std::vector<float> y(traj.points.size());
    for (size_t i = 0; i < traj.points.size(); ++i) {
      x[i] = std::chrono::duration_cast<std::chrono::milliseconds>(traj.points[i].time_from_start).count();
    }
    for (size_t i = 0; i < NJoints; ++i) {
      for (size_t j = 0; j < traj.points.size(); ++j) {
        y[j] = traj.points[j].positions[i];
      }
      interpolators_[i].setData(x, y);
    }
  }

  /**
   * @brief 函数调用运算符 - 查询指定时间的关节位置
   *
   * 根据给定的时间偏移量，查询轨迹上对应时刻的关节位置。
   * 使用样条插值计算各关节在指定时间的位置。
   *
   * @param time_from_start 从轨迹起点开始的时间偏移量
   * @return Point 对应时间点的关节位置数据
   *
   * @note 使用前必须先调用 SetTrajectory 方法设置轨迹数据
   * @warning 此方法不进行边界检查，调用时请确保时间在轨迹范围内
   * @see SetTrajectory
   */
  Point operator()(Duration time_from_start) {
    Point ret;
    ret.velocities.resize(NJoints);
    ret.positions.resize(NJoints);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_from_start).count();
    for (size_t i = 0; i < NJoints; ++i) {
      ret.positions[i] = interpolators_[i](elapsed_ms);
    }
    return ret;
  }

 private:
  std::array<Spline1D<float>, NJoints> interpolators_;
};
