// 第二阶段：降自由度 ICP 精配准（冻结旋转，只优化平移）。
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>

#include <optional>

#include "amorph_vision_cpp/types.hpp"

namespace amorph::vision {

/// 从深度图提取掩膜区域的 3D 点（相机系，米）。
pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractMaskedPoints(const cv::Mat& depth_mm, const cv::Mat& mask,
                                                        const Intrinsics& intrinsics);

struct IcpResult {
  Pose pose;
  double fitness = 0.0;
  double rmse = 0.0;
};

/// 3-DOF：固定旋转，只优化 xyz 平移（点对面 + Huber）。
class TranslationRefiner {
 public:
  explicit TranslationRefiner(double max_dist = 0.05, int max_iter = 15, double huber_k = 0.01, int max_points = 800,
                              double voxel_size = 0.004, double tol = 1e-5)
      : max_dist_(max_dist),
        max_iter_(max_iter),
        huber_k_(huber_k),
        max_points_(max_points),
        voxel_size_(voxel_size),
        tol_(tol) {}

  std::optional<IcpResult> Refine(const pcl::PointCloud<pcl::PointXYZ>& source,
                                  const pcl::PointCloud<pcl::PointXYZ>& target, const Pose& init_pose) const;

 private:
  double max_dist_;
  int max_iter_;
  double huber_k_;
  int max_points_;
  double voxel_size_;
  double tol_;
};

/// 1-DOF：沿相机光心→PCA 位置的射线优化标量偏移（3-DOF 不收敛时的回退）。
class RayOffsetRefiner {
 public:
  explicit RayOffsetRefiner(double max_dist = 0.05, int max_iter = 15, double huber_k = 0.01, int max_points = 800,
                            double voxel_size = 0.004, double tol = 1e-5)
      : max_dist_(max_dist),
        max_iter_(max_iter),
        huber_k_(huber_k),
        max_points_(max_points),
        voxel_size_(voxel_size),
        tol_(tol) {}

  std::optional<IcpResult> Refine(const pcl::PointCloud<pcl::PointXYZ>& source,
                                  const pcl::PointCloud<pcl::PointXYZ>& target, const Pose& init_pose) const;

 private:
  double max_dist_;
  int max_iter_;
  double huber_k_;
  int max_points_;
  double voxel_size_;
  double tol_;
};

/// 用降自由度 ICP 精化 PCA 位姿：先 3-DOF，fitness 不足时回退到 1-DOF。
/// 旋转始终冻结在 PCA 结果上——能量单元可见面接近对称，放开旋转会让
/// 平移分量去补偿旋转误差。
Pose RefinePose(const pcl::PointCloud<pcl::PointXYZ>& source, const pcl::PointCloud<pcl::PointXYZ>& target,
                const Pose& init_pose);

}  // namespace amorph::vision
