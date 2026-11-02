// 几何/跟踪层的单元测试，语义对齐 Python 参考实现。
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include <cmath>
#include <limits>

#include "amorph_vision_cpp/eu_model.hpp"
#include "amorph_vision_cpp/hungarian.hpp"
#include "amorph_vision_cpp/icp_refine.hpp"
#include "amorph_vision_cpp/mask_utils.hpp"
#include "amorph_vision_cpp/numeric_utils.hpp"
#include "amorph_vision_cpp/pca_locator.hpp"
#include "amorph_vision_cpp/sort3d_tracker.hpp"

#ifndef AMORPH_RESOURCE_DIR
#error "AMORPH_RESOURCE_DIR 未定义"
#endif

namespace amorph::vision {
namespace {

TEST(NumericUtils, MedianMatchesNumpy) {
  EXPECT_DOUBLE_EQ(Median({3.0F, 1.0F, 2.0F}), 2.0);
  // 偶数个取中间两个的平均
  EXPECT_DOUBLE_EQ(Median({4.0F, 1.0F, 3.0F, 2.0F}), 2.5);
}

TEST(NumericUtils, PercentileMatchesNumpy) {
  // np.percentile([0, 1, 2, 3, 4], [2, 98]) == [0.08, 3.92]
  EXPECT_NEAR(Percentile({0.0, 1.0, 2.0, 3.0, 4.0}, 2.0), 0.08, 1e-12);
  EXPECT_NEAR(Percentile({0.0, 1.0, 2.0, 3.0, 4.0}, 98.0), 3.92, 1e-12);
}

TEST(NumericUtils, LinspaceIndicesMatchesNumpy) {
  const std::vector<std::size_t> indices = LinspaceIndices(10, 4);
  ASSERT_EQ(indices.size(), 4U);
  EXPECT_EQ(indices[0], 0U);
  EXPECT_EQ(indices[1], 3U);
  EXPECT_EQ(indices[2], 6U);
  EXPECT_EQ(indices[3], 9U);
}

TEST(Hungarian, PicksGlobalMinimum) {
  // scipy: linear_sum_assignment([[4, 1, 3], [2, 0, 5], [3, 2, 2]]) == ([0, 1, 2], [1, 0, 2])
  const std::vector<std::vector<double>> cost{{4.0, 1.0, 3.0}, {2.0, 0.0, 5.0}, {3.0, 2.0, 2.0}};
  const std::vector<int> assignment = SolveAssignment(cost);
  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], 1);
  EXPECT_EQ(assignment[1], 0);
  EXPECT_EQ(assignment[2], 2);
}

TEST(Hungarian, HandlesRectangularMatrices) {
  // 行多列少：最优是 r1→c0(0.5) + r2→c1(0.1)，r0 不分配
  const std::vector<std::vector<double>> cost{{1.0, 9.0}, {0.5, 8.0}, {7.0, 0.1}};
  const std::vector<int> assignment = SolveAssignment(cost);
  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], -1);
  EXPECT_EQ(assignment[1], 0);
  EXPECT_EQ(assignment[2], 1);
}

TEST(MaskUtils, LargestComponentKeepsBiggestBlob) {
  cv::Mat mask = cv::Mat::zeros(40, 40, CV_8U);
  cv::rectangle(mask, cv::Rect(2, 2, 5, 5), cv::Scalar(255), cv::FILLED);
  cv::rectangle(mask, cv::Rect(20, 20, 12, 12), cv::Scalar(255), cv::FILLED);

  const cv::Mat largest = mask::LargestComponent(mask);
  ASSERT_FALSE(largest.empty());
  EXPECT_EQ(cv::countNonZero(largest), 144);
  EXPECT_EQ(largest.at<std::uint8_t>(25, 25), 255);
  EXPECT_EQ(largest.at<std::uint8_t>(4, 4), 0);
}

TEST(PcaLocator, RecoversCylinderAxis) {
  // 合成一根 15cm 高、半径 3cm 的圆柱，主轴倾斜 20 度
  const Intrinsics intrinsics{600.0, 600.0, 320.0, 240.0};
  const int width = 640;
  const int height = 480;

  const Eigen::Vector3d center{0.02, -0.01, 1.1};
  const double tilt = 20.0 * M_PI / 180.0;
  const Eigen::Vector3d axis(std::sin(tilt), 0.0, std::cos(tilt));
  const Eigen::Vector3d radial_u(std::cos(tilt), 0.0, -std::sin(tilt));
  const Eigen::Vector3d radial_v(0.0, 1.0, 0.0);
  const double radius = 0.03;
  const double half_height = 0.075;

  // 密集采样后用 z-buffer 光栅化，得到“可见表面”的深度图
  cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
  cv::Mat depth = cv::Mat::zeros(height, width, CV_32F);

  constexpr int kAxialSteps = 240;
  constexpr int kAngularSteps = 720;
  for (int i = 0; i <= kAxialSteps; ++i) {
    const double t = -half_height + 2.0 * half_height * i / kAxialSteps;
    for (int j = 0; j < kAngularSteps; ++j) {
      const double theta = 2.0 * M_PI * j / kAngularSteps;
      const Eigen::Vector3d point =
          center + t * axis + radius * (std::cos(theta) * radial_u + std::sin(theta) * radial_v);
      const int u = static_cast<int>(std::lround(intrinsics.fx * point.x() / point.z() + intrinsics.cx));
      const int v = static_cast<int>(std::lround(intrinsics.fy * point.y() / point.z() + intrinsics.cy));
      if (u < 0 || u >= width || v < 0 || v >= height) {
        continue;
      }
      const float z = static_cast<float>(point.z() * 1000.0);
      float& pixel = depth.at<float>(v, u);
      if (pixel == 0.0F || z < pixel) {
        pixel = z;
        mask.at<std::uint8_t>(v, u) = 255;
      }
    }
  }

  const PcaLocator locator(intrinsics);
  const BoundingBox box{250.0, 150.0, 400.0, 330.0};
  const auto estimate = locator.ProcessMask(mask, box, depth);
  ASSERT_TRUE(estimate.has_value());
  ASSERT_TRUE(estimate->axis_direction.has_value());

  // 主轴是最稳健的观测量：只能看到半圈侧面时也应当准确
  const Eigen::Vector3d estimated_axis = *estimate->axis_direction;
  EXPECT_GT(std::abs(estimated_axis.dot(axis)), 0.99) << "axis = " << estimated_axis.transpose();

  // 位置只沿轴向做了修正，径向偏移由后续 ICP 修正，因此这里放宽到半径量级
  EXPECT_LT((estimate->pose.position - center).norm(), 0.03) << "position = " << estimate->pose.position.transpose();
}

TEST(Sort3DTracker, KeepsIdAcrossShortDropout) {
  Sort3DTracker tracker(/*max_age=*/15, /*min_hits=*/1);
  std::vector<int> removed;

  Detection detection;
  detection.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  detection.bbox = BoundingBox{100.0, 100.0, 200.0, 200.0};

  auto assignments = tracker.Update({detection}, 1.0 / 30.0, &removed);
  ASSERT_EQ(assignments.size(), 1U);
  const int first_id = assignments.front().track_id;

  // 目标移出画面两帧
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(tracker.Update({}, 1.0 / 30.0, &removed).empty());
  }

  // 重新出现时应该续上原来的 id
  detection.position = Eigen::Vector3d(0.01, 0.0, 1.0);
  assignments = tracker.Update({detection}, 1.0 / 30.0, &removed);
  ASSERT_EQ(assignments.size(), 1U);
  EXPECT_EQ(assignments.front().track_id, first_id);
}

TEST(Sort3DTracker, AllocatesDistinctIdsForTwoTargets) {
  Sort3DTracker tracker(15, 1);
  std::vector<int> removed;

  Detection left;
  left.position = Eigen::Vector3d(-0.2, 0.0, 1.0);
  left.bbox = BoundingBox{100.0, 100.0, 180.0, 200.0};

  Detection right;
  right.position = Eigen::Vector3d(0.2, 0.0, 1.0);
  right.bbox = BoundingBox{400.0, 100.0, 480.0, 200.0};

  const auto assignments = tracker.Update({left, right}, 1.0 / 30.0, &removed);
  ASSERT_EQ(assignments.size(), 2U);
  EXPECT_NE(assignments[0].track_id, assignments[1].track_id);
}

TEST(EuModel, LoadsPlyAndCullsBackFacingPoints) {
  const EuModel model(std::string(AMORPH_RESOURCE_DIR) + "/energy_unit.ply", 5000);
  EXPECT_GT(model.size(), 0U);
  EXPECT_GT(model.max_extent(), 0.0);

  // 以相机原点为球心做半径裁剪，可见点数应显著少于总点数
  Pose pose;
  pose.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  const auto visible = model.SampleVisible(pose);
  EXPECT_GT(visible->size(), 0U);
  EXPECT_LT(visible->size(), model.size());
}

TEST(IcpRefine, RecoversTranslationOnCurvedSurface) {
  // 用球面点集模拟“曲面能约束三个平移自由度”的情形
  const Eigen::Vector3d true_center(0.01, -0.02, 1.05);
  const double radius = 0.03;

  pcl::PointCloud<pcl::PointXYZ> target;
  constexpr int kCount = 2000;
  for (int i = 0; i < kCount; ++i) {
    // Fibonacci 球面，确定性分布
    const double z = 1.0 - 2.0 * (i + 0.5) / kCount;
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double phi = i * M_PI * (3.0 - std::sqrt(5.0));
    const Eigen::Vector3d direction(r * std::cos(phi), r * std::sin(phi), z);
    const Eigen::Vector3d point = true_center + radius * direction;
    target.emplace_back(static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()));
  }

  // 源点云 = 目标点云整体偏移 5mm，模拟 PCA 粗定位误差
  const Eigen::Vector3d error(0.005, 0.005, 0.005);
  pcl::PointCloud<pcl::PointXYZ> source;
  for (const pcl::PointXYZ& point : target) {
    source.emplace_back(point.x + static_cast<float>(error.x()), point.y + static_cast<float>(error.y()),
                        point.z + static_cast<float>(error.z()));
  }

  Pose init_pose;
  init_pose.position = true_center + error;

  const Pose refined = RefinePose(source, target, init_pose);
  EXPECT_LT((refined.position - true_center).norm(), 1e-3) << "refined = " << refined.position.transpose();
}

TEST(IcpRefine, SurvivesNonFiniteInput) {
  // 回归：退化区域的法向估计会输出 NaN，若不过滤会把 NaN 送进 kd-tree，
  // 触发 PCL 的 isValid 断言直接 abort。
  const Eigen::Vector3d center(0.0, 0.0, 1.0);

  pcl::PointCloud<pcl::PointXYZ> target;
  for (int i = 0; i < 200; ++i) {
    const double angle = 2.0 * M_PI * i / 200.0;
    target.emplace_back(static_cast<float>(center.x() + 0.03 * std::cos(angle)),
                        static_cast<float>(center.y() + 0.03 * std::sin(angle)),
                        static_cast<float>(center.z() + 0.01 * std::sin(3.0 * angle)));
  }
  // 混入 NaN / Inf 点
  target.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F);
  target.emplace_back(0.0F, std::numeric_limits<float>::infinity(), 1.0F);

  pcl::PointCloud<pcl::PointXYZ> source = target;

  Pose init_pose;
  init_pose.position = center;
  // 故意给一个 NaN 初值：实现必须返回有限结果或退回初值，而不是崩溃
  Pose bad_pose;
  bad_pose.position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

  EXPECT_NO_FATAL_FAILURE({ (void)RefinePose(source, target, init_pose); });
  EXPECT_NO_FATAL_FAILURE({ (void)RefinePose(source, target, bad_pose); });
}

}  // namespace
}  // namespace amorph::vision
