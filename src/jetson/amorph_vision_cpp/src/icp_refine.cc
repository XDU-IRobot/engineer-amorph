#include "amorph_vision_cpp/icp_refine.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <vector>

#include "amorph_vision_cpp/mask_utils.hpp"
#include "amorph_vision_cpp/numeric_utils.hpp"

namespace amorph::vision {
namespace {

constexpr std::size_t kMinIcpPoints = 10;
constexpr double kFitnessThreshold = 0.3;

/// Eigen 向量的所有分量都是有限值。
template <typename Derived>
bool IsFinite(const Eigen::MatrixBase<Derived>& vector) {
  return vector.allFinite();
}

/// 体素降采样后的目标点云及其法向，整个迭代过程复用。
struct PreparedTarget {
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<Eigen::Vector3d> normals;
};

pcl::PointCloud<pcl::PointXYZ> Subsample(const pcl::PointCloud<pcl::PointXYZ>& cloud, int max_points) {
  // 等间隔抽稀前先剔除非有限点，保证送到 kd-tree 的查询点全部有效
  auto finite = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  finite->reserve(cloud.size());
  for (const pcl::PointXYZ& point : cloud) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }

  if (max_points <= 0 || finite->size() <= static_cast<std::size_t>(max_points)) {
    return *finite;
  }
  pcl::PointCloud<pcl::PointXYZ> out;
  const std::vector<std::size_t> indices =
      LinspaceIndices(finite->size(), static_cast<std::size_t>(max_points));
  out.reserve(indices.size());
  for (const std::size_t index : indices) {
    out.push_back((*finite)[index]);
  }
  return out;
}

bool PrepareTarget(const pcl::PointCloud<pcl::PointXYZ>& cloud, double voxel_size, PreparedTarget* target) {
  // PCL 的 kd-tree 有 isValid 断言，而 NormalEstimation 内部就靠 radiusSearch 工作，
  // 所以 NaN/Inf 必须在进入任何 PCL 组件之前就清掉——放在法向估计之后为时已晚。
  auto finite = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  finite->reserve(cloud.size());
  for (const pcl::PointXYZ& point : cloud) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  if (finite->size() < kMinIcpPoints) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (voxel_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(finite);
    voxel.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size), static_cast<float>(voxel_size));
    voxel.filter(*downsampled);
  } else {
    *downsampled = *finite;
  }
  if (downsampled->size() < kMinIcpPoints) {
    return false;
  }

  // 与参考实现一致：局部邻域半径 2cm（PCL 不支持 open3d 的 radius+max_nn 组合，
  // 这里只给半径；体素已经降采样到 4mm，邻域点数不会失控）
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  normal_estimator.setInputCloud(downsampled);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setRadiusSearch(0.02);
  pcl::PointCloud<pcl::Normal> normals;
  normal_estimator.compute(normals);
  if (normals.size() != downsampled->size()) {
    return false;
  }

  // 法向估计在退化区域（邻域点过少、近共线、平面无厚度）会输出 NaN。
  // 这些 NaN 一旦参与 residual 计算就会污染步长，最终把 NaN 作为查询点送进
  // kd-tree，触发 PCL 的 isValid 断言。因此这里把无效点整体剔除，
  // 并保证 points 与 normals 严格对齐。
  target->points.clear();
  target->normals.clear();
  target->points.reserve(downsampled->size());
  target->normals.reserve(downsampled->size());
  for (std::size_t i = 0; i < normals.size(); ++i) {
    const Eigen::Vector3d normal(normals[i].normal_x, normals[i].normal_y, normals[i].normal_z);
    if (!IsFinite(normal) || normal.norm() < 0.1) {
      continue;
    }
    const Eigen::Vector3d point = (*downsampled)[i].getVector3fMap().cast<double>();
    if (!IsFinite(point)) {
      continue;
    }
    target->points.push_back((*downsampled)[i]);
    target->normals.push_back(normal.normalized());
  }
  return target->points.size() >= kMinIcpPoints;
}

/// 加权点对面残差的 3x3 法方程解（对应 Python 版 _solve_translation）。
Eigen::Vector3d SolveTranslation(const std::vector<Eigen::Vector3d>& normals, const std::vector<double>& residuals,
                                 const std::vector<double>& weights) {
  Eigen::Matrix3d normal_matrix = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < normals.size(); ++i) {
    const Eigen::Vector3d weighted = weights[i] * normals[i];
    normal_matrix += weighted * weighted.transpose();
    rhs += weighted * (residuals[i] * weights[i]);
  }
  normal_matrix += 1e-12 * Eigen::Matrix3d::Identity();
  return normal_matrix.ldlt().solve(rhs);
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractMaskedPoints(const cv::Mat& depth_mm, const cv::Mat& mask,
                                                        const Intrinsics& intrinsics) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (depth_mm.empty() || mask.empty() || depth_mm.size() != mask.size()) {
    return cloud;
  }
  for (const cv::Point& pixel : mask::ForegroundPixels(mask)) {
    const float depth = depth_mm.at<float>(pixel.y, pixel.x);
    // 写成 !(depth > 0) 而不是 depth <= 0：NaN 与任何值比较都是 false，
    // 后者会让 NaN 深度涓进点云，进而污染 ICP。
    if (!(depth > 0.0F)) {
      continue;
    }
    const double z = static_cast<double>(depth) / 1000.0;
    pcl::PointXYZ point;
    point.x = static_cast<float>((pixel.x - intrinsics.cx) * z / intrinsics.fx);
    point.y = static_cast<float>((pixel.y - intrinsics.cy) * z / intrinsics.fy);
    point.z = static_cast<float>(z);
    if (!pcl::isFinite(point)) {
      continue;
    }
    cloud->push_back(point);
  }
  return cloud;
}

std::optional<IcpResult> TranslationRefiner::Refine(const pcl::PointCloud<pcl::PointXYZ>& source,
                                                    const pcl::PointCloud<pcl::PointXYZ>& target,
                                                    const Pose& init_pose) const {
  if (source.size() < kMinIcpPoints || target.size() < kMinIcpPoints) {
    return std::nullopt;
  }

  PreparedTarget prepared;
  if (!PrepareTarget(target, voxel_size_, &prepared)) {
    return std::nullopt;
  }

  const pcl::PointCloud<pcl::PointXYZ> src = Subsample(source, max_points_);
  if (src.empty()) {
    return std::nullopt;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(prepared.points));

  const Eigen::Vector3d t_pca = init_pose.position;
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  double rmse = std::numeric_limits<double>::infinity();
  double fitness = 0.0;

  std::vector<int> nn_index(1);
  std::vector<float> nn_distance(1);
  const double max_dist_sq = max_dist_ * max_dist_;

  for (int iter = 0; iter < max_iter_; ++iter) {
    // 上一轮步长若已经发散，后续查询点必然非有限，直接放弃
    if (!IsFinite(delta)) {
      return std::nullopt;
    }

    std::vector<Eigen::Vector3d> normals;
    std::vector<double> residuals;
    std::vector<double> weights;
    normals.reserve(src.size());
    residuals.reserve(src.size());
    weights.reserve(src.size());

    double squared_sum = 0.0;
    for (const pcl::PointXYZ& point : src) {
      pcl::PointXYZ moved;
      moved.x = static_cast<float>(point.x + delta.x());
      moved.y = static_cast<float>(point.y + delta.y());
      moved.z = static_cast<float>(point.z + delta.z());
      // PCL 的 kd-tree 内部有 isValid 断言（Non-finite 会 abort），必须自己拦
      if (!pcl::isFinite(moved)) {
        continue;
      }
      if (tree.nearestKSearch(moved, 1, nn_index, nn_distance) <= 0) {
        continue;
      }
      if (nn_distance[0] > max_dist_sq) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(nn_index[0]);
      if (index >= prepared.points.size() || index >= prepared.normals.size()) {
        continue;
      }
      const Eigen::Vector3d target_point = prepared.points[index].getVector3fMap().cast<double>();
      const Eigen::Vector3d normal = prepared.normals[index];
      const double residual = (target_point - moved.getVector3fMap().cast<double>()).dot(normal);
      normals.push_back(normal);
      residuals.push_back(residual);
      weights.push_back(HuberWeight(residual, huber_k_));
      squared_sum += residual * residual;
    }

    if (normals.size() < kMinIcpPoints) {
      break;
    }

    const Eigen::Vector3d step = SolveTranslation(normals, residuals, weights);
    if (!IsFinite(step)) {
      return std::nullopt;
    }
    delta += step;

    fitness = static_cast<double>(normals.size()) / static_cast<double>(src.size());
    rmse = std::sqrt(squared_sum / static_cast<double>(normals.size()));
    if (step.norm() < tol_) {
      break;
    }
  }

  const Eigen::Vector3d refined_position = t_pca + delta;
  if (!IsFinite(refined_position)) {
    return std::nullopt;
  }

  IcpResult result;
  result.pose.position = refined_position;
  result.pose.orientation = init_pose.orientation;
  result.fitness = fitness;
  result.rmse = rmse;
  return result;
}

std::optional<IcpResult> RayOffsetRefiner::Refine(const pcl::PointCloud<pcl::PointXYZ>& source,
                                                  const pcl::PointCloud<pcl::PointXYZ>& target,
                                                  const Pose& init_pose) const {
  if (source.size() < kMinIcpPoints || target.size() < kMinIcpPoints) {
    return std::nullopt;
  }

  const Eigen::Vector3d t_pca = init_pose.position;
  const double radius = t_pca.norm();
  if (radius < 1e-6) {
    return std::nullopt;
  }
  const Eigen::Vector3d direction = t_pca / radius;

  PreparedTarget prepared;
  if (!PrepareTarget(target, voxel_size_, &prepared)) {
    return std::nullopt;
  }

  const pcl::PointCloud<pcl::PointXYZ> src = Subsample(source, max_points_);
  if (src.empty()) {
    return std::nullopt;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(prepared.points));

  double lambda = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
  double fitness = 0.0;

  std::vector<int> nn_index(1);
  std::vector<float> nn_distance(1);
  const double max_dist_sq = max_dist_ * max_dist_;

  for (int iter = 0; iter < max_iter_; ++iter) {
    double numerator = 0.0;
    double denominator = 0.0;
    double squared_sum = 0.0;
    std::size_t valid = 0;

    for (const pcl::PointXYZ& point : src) {
      const Eigen::Vector3d moved = point.getVector3fMap().cast<double>() + lambda * direction;
      if (!IsFinite(moved)) {
        continue;
      }
      pcl::PointXYZ query;
      query.x = static_cast<float>(moved.x());
      query.y = static_cast<float>(moved.y());
      query.z = static_cast<float>(moved.z());
      if (!pcl::isFinite(query)) {
        continue;
      }
      if (tree.nearestKSearch(query, 1, nn_index, nn_distance) <= 0) {
        continue;
      }
      if (nn_distance[0] > max_dist_sq) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(nn_index[0]);
      if (index >= prepared.points.size() || index >= prepared.normals.size()) {
        continue;
      }
      const Eigen::Vector3d normal = prepared.normals[index];
      const double residual = (prepared.points[index].getVector3fMap().cast<double>() - moved).dot(normal);
      const double projection = direction.dot(normal);
      const double weight = HuberWeight(residual, huber_k_);

      numerator += weight * projection * residual;
      denominator += weight * projection * projection;
      squared_sum += residual * residual;
      ++valid;
    }

    if (valid < kMinIcpPoints || std::abs(denominator) < 1e-12) {
      break;
    }

    const double step = numerator / denominator;
    if (!std::isfinite(step)) {
      return std::nullopt;
    }
    lambda += step;

    fitness = static_cast<double>(valid) / static_cast<double>(src.size());
    rmse = std::sqrt(squared_sum / static_cast<double>(valid));
    if (std::abs(step) * radius < tol_) {
      break;
    }
  }

  const Eigen::Vector3d refined_position = t_pca + lambda * direction;
  if (!IsFinite(refined_position)) {
    return std::nullopt;
  }

  IcpResult result;
  result.pose.position = refined_position;
  result.pose.orientation = init_pose.orientation;
  result.fitness = fitness;
  result.rmse = rmse;
  return result;
}

Pose RefinePose(const pcl::PointCloud<pcl::PointXYZ>& source, const pcl::PointCloud<pcl::PointXYZ>& target,
                const Pose& init_pose) {
  if (source.size() < 50 || target.size() < 50) {
    return init_pose;
  }

  if (const auto result = TranslationRefiner().Refine(source, target, init_pose);
      result.has_value() && result->fitness > kFitnessThreshold) {
    return result->pose;
  }

  if (const auto result = RayOffsetRefiner().Refine(source, target, init_pose); result.has_value()) {
    return result->pose;
  }
  return init_pose;
}

}  // namespace amorph::vision
