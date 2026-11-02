// 能量单元模型：从 PLY 采样，并按位姿裁出面向相机的可见表面。
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <string>
#include <vector>

#include "amorph_vision_cpp/types.hpp"

namespace amorph::vision {

class EuModel {
 public:
  /// 从 PLY 网格按面积加权采样并归中到原点。
  explicit EuModel(const std::string& ply_path, std::size_t num_points = 50000);

  /// 把模型按位姿摆到相机系，并以相机原点为球心剔除背面/不可见点。
  /// @param clip_factor 裁剪半径系数，半径 = clip_factor * |position|。
  pcl::PointCloud<pcl::PointXYZ>::Ptr SampleVisible(const Pose& pose, double clip_factor = 0.975) const;

  std::size_t size() const { return points_.size(); }
  double max_extent() const { return max_extent_; }

 private:
  pcl::PointCloud<pcl::PointXYZ> points_;  ///< 已归中的模型采样点
  double max_extent_ = 0.0;
};

}  // namespace amorph::vision
