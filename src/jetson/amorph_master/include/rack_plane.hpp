// 架面拟合：能量单元的 z 轴一般共面（都插在同一个架子上），用 RANSAC 拟出这个平面。
#pragma once

#include <optional>

#include <Eigen/Core>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace amorph_master {

/// @brief 拟合出的平面，center 取内点质心
struct Plane {
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
};

/// @brief 用检测结果里各能量单元的位姿拟合它们 z 轴所在的平面。
/// 每个单元取中心点与沿 z 轴的一点参与拟合，单排共线的单元也能定出平面。
/// @param detections 检测结果，位姿需在其自身（相机光学）坐标系下，原点即相机位置
/// @return 少于两个单元或三点全部共线时返回 nullopt；法向量总是指向相机
std::optional<Plane> FitRackPlane(const vision_msgs::msg::Detection3DArray &detections);

}  // namespace amorph_master
