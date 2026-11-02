#include "rack_plane.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace amorph_master {
namespace {

constexpr double kAxisSampleLength = 0.05;  ///< 沿 z 轴采样时取的长度，单位米
constexpr double kInlierDistance = 0.01;    ///< RANSAC 内点距离阈值，单位米
constexpr double kMinCrossNorm = 1e-3;      ///< 三点叉积小于此值视为共线，跳过，单位平方米

/// @brief 用检测结果里的位姿构造参与拟合的点。每个单元取中心点与沿 z 轴的一点。
std::vector<Eigen::Vector3d> BuildPoints(const vision_msgs::msg::Detection3DArray &detections) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(detections.detections.size() * 2);
  for (const vision_msgs::msg::Detection3D &detection : detections.detections) {
    const geometry_msgs::msg::Pose &pose = detection.bbox.center;
    const Eigen::Vector3d position{pose.position.x, pose.position.y, pose.position.z};
    const Eigen::Quaterniond orientation{pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z};
    points.push_back(position);
    points.push_back(position + kAxisSampleLength * orientation.toRotationMatrix().col(2));
  }
  return points;
}

/// @brief 枚举所有三点组合求最大一致集，再用内点做最小二乘精化。
std::optional<Plane> FitPlane(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_offset = 0.0;
  int best_inliers = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      for (std::size_t k = j + 1; k < points.size(); ++k) {
        const Eigen::Vector3d cross = (points[j] - points[i]).cross(points[k] - points[i]);
        // 三点接近共线时法向不可信
        if (cross.norm() < kMinCrossNorm) {
          continue;
        }
        const Eigen::Vector3d normal = cross.normalized();
        const double offset = normal.dot(points[i]);
        const int inliers =
            static_cast<int>(std::count_if(points.begin(), points.end(), [&](const Eigen::Vector3d &point) {
              return std::abs(normal.dot(point) - offset) < kInlierDistance;
            }));
        if (inliers > best_inliers) {
          best_inliers = inliers;
          best_normal = normal;
          best_offset = offset;
        }
      }
    }
  }
  if (best_inliers == 0) {
    return std::nullopt;
  }

  // 三点定的平面受噪声影响大，用最大一致集重算质心与法向（协方差最小特征值方向）
  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(points.size());
  for (const Eigen::Vector3d &point : points) {
    if (std::abs(best_normal.dot(point) - best_offset) < kInlierDistance) {
      inliers.push_back(point);
    }
  }
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &point : inliers) {
    center += point;
  }
  center /= static_cast<double>(inliers.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d &point : inliers) {
    const Eigen::Vector3d delta = point - center;
    covariance += delta * delta.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{covariance};
  return Plane{.center = center, .normal = solver.eigenvectors().col(0)};
}

}  // namespace

std::optional<Plane> FitRackPlane(const vision_msgs::msg::Detection3DArray &detections) {
  // 一个单元只能确定一条 z 轴，定不出平面
  if (detections.detections.size() < 2) {
    return std::nullopt;
  }
  std::optional<Plane> plane = FitPlane(BuildPoints(detections));
  if (!plane.has_value()) {
    return plane;
  }
  // 检测结果在相机光学系下，原点就是相机；法向量一律指回相机，避免逐帧正负翻转
  if (plane->normal.dot(plane->center) > 0.0) {
    plane->normal = -plane->normal;
  }
  return plane;
}

}  // namespace amorph_master
