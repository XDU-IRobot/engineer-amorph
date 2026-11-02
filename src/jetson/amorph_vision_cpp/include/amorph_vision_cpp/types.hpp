// 能量单元定位的基础数据类型。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <optional>

namespace amorph::vision {

/// 针孔相机内参，单位像素。
struct Intrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

/// 相机系下的 6D 位姿。
struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/// 像素坐标系下 [x1, y1, x2, y2] 的轴对齐框。
struct BoundingBox {
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;

  /// 与 Python 版 `_iou_xyxy` 一致的交并比。
  double Iou(const BoundingBox& other) const {
    const double inter_x1 = std::max(x1, other.x1);
    const double inter_y1 = std::max(y1, other.y1);
    const double inter_x2 = std::min(x2, other.x2);
    const double inter_y2 = std::min(y2, other.y2);

    const double inter_w = std::max(0.0, inter_x2 - inter_x1);
    const double inter_h = std::max(0.0, inter_y2 - inter_y1);
    const double inter_area = inter_w * inter_h;

    const double area_a = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
    const double area_b = std::max(0.0, other.x2 - other.x1) * std::max(0.0, other.y2 - other.y1);
    const double uni = area_a + area_b - inter_area;

    if (uni <= 1e-9) {
      return 0.0;
    }
    return inter_area / uni;
  }
};

/// PCA + ICP 的位姿估计结果。
struct PoseEstimate {
  Pose pose;
  /// 圆柱主轴方向（相机系单位向量）；点数不足时为 nullopt。
  std::optional<Eigen::Vector3d> axis_direction;
  /// 深度约束后的二值掩膜，CV_8UC1，前景 255。
  cv::Mat binary_mask;
};

/// 单个 2D 检测 + 3D 位姿，供跟踪器关联使用。
struct Detection {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  BoundingBox bbox;
  float confidence = 0.0F;
  int class_id = 0;
  /// 全分辨率二值掩膜，CV_8UC1，前景 255。
  cv::Mat mask;
  /// 该检测在原始 YOLO 输出中的下标，用于把 track id 写回标注。
  int annotation_index = -1;
};

/// 跟踪输出：某个 track_id 关联到当前帧的检测下标。
struct TrackAssignment {
  int track_id = 0;
  int detection_index = 0;
  /// 卡尔曼滤波后的位置（相机系）。KF 只估位置与速度，不含姿态。
  Eigen::Vector3d filtered_position = Eigen::Vector3d::Zero();
};

}  // namespace amorph::vision
