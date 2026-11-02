// 调试可视化：foxglove 图像标注 + 位姿估计过程的中间点云。
#pragma once

#include <foxglove_msgs/msg/image_annotations.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "amorph_vision_cpp/publisher_pool.hpp"
#include "amorph_vision_cpp/types.hpp"
#include "amorph_vision_cpp/yolo_segmenter.hpp"

namespace amorph::vision {

struct AnnotationStyle {
  std::array<double, 4> mask_outline_color{0.0, 1.0, 0.0, 1.0};
  std::array<double, 4> mask_fill_color{0.0, 1.0, 0.0, 0.25};
  std::array<double, 4> box_color{1.0, 0.5, 0.0, 1.0};
  std::array<double, 4> text_color{1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> text_background_color{0.0, 0.0, 0.0, 0.7};
  double line_thickness = 2.0;
  double font_size = 12.0;
};

/// 构造 ImageAnnotations：掩膜轮廓 + 检测框 + 文本（含 track id）。
///
/// Foxglove 依靠时间戳把标注叠加到对应帧的图像上，因此 timestamp 必须来自图像头。
/// track_ids 与 detections 一一对应，值为 -1 表示该检测没有 track id。
foxglove_msgs::msg::ImageAnnotations BuildAnnotations(const std::vector<SegmentDetection>& detections,
                                                      const std::vector<int>& track_ids,
                                                      const std::vector<std::string>& class_names,
                                                      const builtin_interfaces::msg::Time& stamp,
                                                      const AnnotationStyle& style = {});

/// 单个目标在位姿估计中的中间点云（相机光学系，米）。
///
/// 三组点云叠看即可定位问题：model_pca 与 scene_points 整体偏移说明 PCA 粗定位有偏差，
/// model_icp 与 scene_points 不贴合说明 ICP 未收敛。
struct PoseDebugClouds {
  pcl::PointCloud<pcl::PointXYZ> scene_points;  ///< 掩膜内深度点，即 ICP 的 target
  pcl::PointCloud<pcl::PointXYZ> model_pca;     ///< 模型按 PCA 位姿摆放后的可见点
  pcl::PointCloud<pcl::PointXYZ> model_icp;     ///< 模型按 ICP 位姿摆放后的可见点
};

/// 调试点云发布池，话题为 debug_clouds/det<index>/{scene,model_pca,model_icp}。
///
/// 发布器懒创建并缓存；目标是 detection 在 YOLO 输出中的下标，不会超过 engine 的 max_dets。
class DebugCloudPool {
 public:
  /// @param stride 抽稀步长，用于控制带宽（<= 0 视为 1）
  DebugCloudPool(rclcpp::Node& node, std::string topic_prefix, int stride)
      : pool_(node, rclcpp::SensorDataQoS()), topic_prefix_(std::move(topic_prefix)), stride_(stride > 0 ? stride : 1) {}

  void Publish(const PoseDebugClouds& clouds, int detection_index, const std_msgs::msg::Header& header);

 private:
  void PublishOne(const std::string& topic, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                  const std_msgs::msg::Header& header, const std::array<float, 3>& rgb);

  PublisherPool<sensor_msgs::msg::PointCloud2> pool_;
  std::string topic_prefix_;
  int stride_;
};

/// 打包成 PointCloud2（x/y/z/rgb 四个 float32 字段），rgb 按 PCL 约定放在低 24 位。
sensor_msgs::msg::PointCloud2 BuildPointCloud2(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                               const std_msgs::msg::Header& header,
                                               const std::array<float, 3>& rgb, int stride = 1);

}  // namespace amorph::vision
