#include "amorph_vision_cpp/pca_locator.hpp"

#include <Eigen/Eigenvalues>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

#include "amorph_vision_cpp/mask_utils.hpp"
#include "amorph_vision_cpp/numeric_utils.hpp"

namespace amorph::vision {
namespace {

/// 深度离群点阈值，与 Python 版一致（8cm）。
constexpr double kDepthOutlierThresholdM = 0.08;
/// 质心深度采样的邻域半径（像素）。
constexpr int kCentroidSampleRadius = 10;
/// 少于该数目点时放弃估计。
constexpr std::size_t kMinPoints = 10;

}  // namespace

std::optional<PoseEstimate> PcaLocator::ProcessMask(const cv::Mat& mask, const BoundingBox& box,
                                                    const cv::Mat& depth_mm) const {
  if (mask.empty() || depth_mm.empty() || mask.size() != depth_mm.size()) {
    return std::nullopt;
  }

  // 1. 腐蚀去掉边缘
  const cv::Mat eroded = mask::Erode(mask, erosion_kernel_size_, erosion_iterations_);

  // 2. 只保留最大连通域
  const cv::Mat largest = mask::LargestComponent(eroded);
  if (largest.empty()) {
    return std::nullopt;
  }
  const std::vector<cv::Point> pixels = mask::ForegroundPixels(largest);
  if (pixels.size() < kMinPoints) {
    return std::nullopt;
  }

  // 3. 深度约束：剔除偏离中位数过远的点（目标边缘落在背景上的像素）
  double median_mm = 0.0;
  if (!mask::MedianDepth(depth_mm, largest, &median_mm)) {
    return std::nullopt;
  }
  const cv::Mat binary_mask = mask::FilterByDepth(depth_mm, largest, median_mm, kDepthOutlierThresholdM);
  if (cv::countNonZero(binary_mask) < 5) {
    return std::nullopt;
  }

  // 4. 质心与质心深度
  const cv::Point2d centroid = mask::Centroid(mask, box);
  double centroid_depth_mm = 0.0;
  if (!mask::CentroidDepth(depth_mm, binary_mask, centroid, kCentroidSampleRadius, &centroid_depth_mm)) {
    return std::nullopt;
  }
  const double z_m = centroid_depth_mm / 1000.0;

  // 5. 反投影到 3D
  Eigen::Vector3d position{(centroid.x - intrinsics_.cx) * z_m / intrinsics_.fx,
                           (centroid.y - intrinsics_.cy) * z_m / intrinsics_.fy, z_m};

  // 6. PCA 估计主轴方向并修正中心
  PoseEstimate estimate;
  const auto fit = EstimateCylinderOrientation(binary_mask, depth_mm);
  if (fit.has_value()) {
    estimate.pose.orientation = fit->orientation;
    estimate.axis_direction = fit->axis;
    position = fit->corrected_center;
  }
  // 总闸：位姿一旦非有限，下游 ICP 会把 NaN 送进 kd-tree 触发 PCL 断言
  if (!position.allFinite() || !estimate.pose.orientation.coeffs().allFinite()) {
    return std::nullopt;
  }
  estimate.pose.position = position;
  estimate.binary_mask = binary_mask;
  return estimate;
}

std::optional<PcaLocator::CylinderFit> PcaLocator::EstimateCylinderOrientation(const cv::Mat& binary_mask,
                                                                               const cv::Mat& depth_mm) const {
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(cv::countNonZero(binary_mask)));
  for (const cv::Point& pixel : mask::ForegroundPixels(binary_mask)) {
    const double z = static_cast<double>(depth_mm.at<float>(pixel.y, pixel.x)) / 1000.0;
    if (z <= 0.0) {
      continue;
    }
    points.emplace_back((pixel.x - intrinsics_.cx) * z / intrinsics_.fx,
                        (pixel.y - intrinsics_.cy) * z / intrinsics_.fy, z);
  }
  if (points.size() < kMinPoints) {
    return std::nullopt;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : points) {
    centroid += point;
  }
  centroid /= static_cast<double>(points.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d& point : points) {
    const Eigen::Vector3d centered = point - centroid;
    covariance += centered * centered.transpose();
  }
  // np.cov 用 n-1 归一化，缩放不影响特征向量
  covariance /= static_cast<double>(points.size() - 1);

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  // 特征值升序排列，最后一列即最大特征值对应的主轴
  Eigen::Vector3d axis = solver.eigenvectors().col(2).normalized();

  // 强制 y 分量为负，避免上下对称导致的方向跳动
  if (axis.y() > 0.0) {
    axis = -axis;
  }

  // 用轴向投影的分位数中点修正中心，抵消透视导致的轴向偏移
  std::vector<double> projections;
  projections.reserve(points.size());
  for (const Eigen::Vector3d& point : points) {
    projections.push_back((point - centroid).dot(axis));
  }
  const double t_low = Percentile(projections, 2.0);
  const double t_high = Percentile(projections, 98.0);

  CylinderFit fit;
  fit.axis = axis;
  fit.corrected_center = centroid + 0.5 * (t_low + t_high) * axis;
  fit.orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
  fit.orientation.normalize();
  return fit;
}

}  // namespace amorph::vision
