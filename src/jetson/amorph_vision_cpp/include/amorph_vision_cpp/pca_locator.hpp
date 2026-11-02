// 第一阶段：由 RGBD 掩膜用 PCA 粗估圆柱位姿。
#pragma once

#include <opencv2/core.hpp>

#include <optional>

#include "amorph_vision_cpp/types.hpp"

namespace amorph::vision {

class PcaLocator {
 public:
  PcaLocator(Intrinsics intrinsics, int erosion_kernel_size = 5, int erosion_iterations = 1)
      : intrinsics_(intrinsics), erosion_kernel_size_(erosion_kernel_size), erosion_iterations_(erosion_iterations) {}

  /// 由分割掩膜 + 深度图估计位姿。
  ///
  /// @param mask     全分辨率二值掩膜，CV_8UC1，前景非零。
  /// @param box      检测框 [x1, y1, x2, y2, ...]，仅在前 4 个元素被使用。
  /// @param depth_mm 深度图，CV_32FC1，单位毫米。
  std::optional<PoseEstimate> ProcessMask(const cv::Mat& mask, const BoundingBox& box, const cv::Mat& depth_mm) const;

  const Intrinsics& intrinsics() const { return intrinsics_; }

 private:
  /// PCA 主轴、四元数与轴向修正后的中心。
  struct CylinderFit {
    Eigen::Vector3d axis;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d corrected_center;
  };

  std::optional<CylinderFit> EstimateCylinderOrientation(const cv::Mat& binary_mask, const cv::Mat& depth_mm) const;

  Intrinsics intrinsics_;
  int erosion_kernel_size_;
  int erosion_iterations_;
};

}  // namespace amorph::vision
