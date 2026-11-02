#include "amorph_vision_cpp/eu_model.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Geometry>
#include <random>
#include <stdexcept>
#include <string>

namespace amorph::vision {

EuModel::EuModel(const std::string& ply_path, std::size_t num_points) {
  // 用原生 PLY 读取器，不走 pcl/io/vtk_lib_io.h（那个头文件无条件 include VTK）
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPLYFile(ply_path, mesh) < 0) {
    throw std::runtime_error("加载 PLY 模型失败: " + ply_path);
  }

  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  if (vertices.empty() || mesh.polygons.empty()) {
    throw std::runtime_error("PLY 模型没有三角面: " + ply_path);
  }

  // 累积三角面面积，用于面积加权采样（等价于 open3d 的 Poisson 采样目标分布）
  std::vector<double> cumulative_area;
  cumulative_area.reserve(mesh.polygons.size());
  double total_area = 0.0;
  for (const pcl::Vertices& face : mesh.polygons) {
    if (face.vertices.size() < 3) {
      continue;
    }
    const Eigen::Vector3d a = vertices[face.vertices[0]].getVector3fMap().cast<double>();
    const Eigen::Vector3d b = vertices[face.vertices[1]].getVector3fMap().cast<double>();
    const Eigen::Vector3d c = vertices[face.vertices[2]].getVector3fMap().cast<double>();
    total_area += 0.5 * (b - a).cross(c - a).norm();
    cumulative_area.push_back(total_area);
  }
  if (total_area <= 0.0) {
    throw std::runtime_error("PLY 模型三角面面积为 0: " + ply_path);
  }

  std::mt19937 generator(12345);  // 固定种子，保证每次启动采样结果一致
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  points_.resize(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    const double target = uniform(generator) * total_area;
    const auto it = std::lower_bound(cumulative_area.begin(), cumulative_area.end(), target);
    std::size_t face_idx = static_cast<std::size_t>(std::distance(cumulative_area.begin(), it));
    if (face_idx >= mesh.polygons.size()) {
      face_idx = mesh.polygons.size() - 1;
    }
    const pcl::Vertices& face = mesh.polygons[face_idx];

    const Eigen::Vector3d a = vertices[face.vertices[0]].getVector3fMap().cast<double>();
    const Eigen::Vector3d b = vertices[face.vertices[1]].getVector3fMap().cast<double>();
    const Eigen::Vector3d c = vertices[face.vertices[2]].getVector3fMap().cast<double>();

    // 重心坐标采样
    double u = uniform(generator);
    double v = uniform(generator);
    if (u + v > 1.0) {
      u = 1.0 - u;
      v = 1.0 - v;
    }
    const Eigen::Vector3d point = a + u * (b - a) + v * (c - a);
    points_[i].x = static_cast<float>(point.x());
    points_[i].y = static_cast<float>(point.y());
    points_[i].z = static_cast<float>(point.z());
  }

  // 归中到原点，位姿平移即为模型中心
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const pcl::PointXYZ& point : points_) {
    centroid += point.getVector3fMap().cast<double>();
  }
  centroid /= static_cast<double>(points_.size());

  max_extent_ = 0.0;
  for (pcl::PointXYZ& point : points_) {
    point.x = static_cast<float>(point.x - centroid.x());
    point.y = static_cast<float>(point.y - centroid.y());
    point.z = static_cast<float>(point.z - centroid.z());
    max_extent_ = std::max(max_extent_, point.getVector3fMap().cast<double>().norm());
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EuModel::SampleVisible(const Pose& pose, double clip_factor) const {
  const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
  const double clip_radius = clip_factor * pose.position.norm();

  auto visible = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  visible->reserve(points_.size());
  for (const pcl::PointXYZ& point : points_) {
    const Eigen::Vector3d transformed = rotation * point.getVector3fMap().cast<double>() + pose.position;
    if (!transformed.allFinite()) {
      continue;
    }
    if (transformed.norm() <= clip_radius) {
      pcl::PointXYZ output;
      output.x = static_cast<float>(transformed.x());
      output.y = static_cast<float>(transformed.y());
      output.z = static_cast<float>(transformed.z());
      visible->push_back(output);
    }
  }
  return visible;
}

}  // namespace amorph::vision
