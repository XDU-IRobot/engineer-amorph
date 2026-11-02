#include "amorph_vision_cpp/visualization.hpp"

#include <foxglove_msgs/msg/color.hpp>
#include <foxglove_msgs/msg/point2.hpp>
#include <foxglove_msgs/msg/points_annotation.hpp>
#include <foxglove_msgs/msg/text_annotation.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace amorph::vision {
namespace {

// 点云配色取差异明显的三种，便于在 foxglove 里叠看
constexpr std::array<float, 3> kSceneRgb{0.20F, 0.90F, 0.20F};     // 绿：场景深度点
constexpr std::array<float, 3> kModelPcaRgb{1.00F, 0.75F, 0.10F};  // 黄：PCA 初值
constexpr std::array<float, 3> kModelIcpRgb{0.20F, 0.55F, 1.00F};  // 蓝：ICP 结果

foxglove_msgs::msg::Color MakeColor(const std::array<double, 4>& rgba) {
  foxglove_msgs::msg::Color color;
  color.r = rgba[0];
  color.g = rgba[1];
  color.b = rgba[2];
  color.a = rgba[3];
  return color;
}

foxglove_msgs::msg::PointsAnnotation MakePolygon(const std::vector<cv::Point>& contour,
                                                 const std::array<double, 4>& outline,
                                                 const std::array<double, 4>& fill,
                                                 const builtin_interfaces::msg::Time& stamp, double thickness) {
  foxglove_msgs::msg::PointsAnnotation annotation;
  annotation.timestamp = stamp;
  annotation.type = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
  annotation.points.reserve(contour.size());
  for (const cv::Point& point : contour) {
    foxglove_msgs::msg::Point2 ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    annotation.points.push_back(ros_point);
  }
  annotation.outline_color = MakeColor(outline);
  annotation.fill_color = MakeColor(fill);
  annotation.thickness = thickness;
  return annotation;
}

float PackRgb(const std::array<float, 3>& rgb) {
  const auto to_byte = [](float value) {
    const float clamped = std::clamp(value, 0.0F, 1.0F);
    return static_cast<std::uint32_t>(clamped * 255.0F + 0.5F);
  };
  const std::uint32_t packed = (to_byte(rgb[0]) << 16) | (to_byte(rgb[1]) << 8) | to_byte(rgb[2]);
  float result = 0.0F;
  std::memcpy(&result, &packed, sizeof(result));
  return result;
}

}  // namespace

foxglove_msgs::msg::ImageAnnotations BuildAnnotations(const std::vector<SegmentDetection>& detections,
                                                      const std::vector<int>& track_ids,
                                                      const std::vector<std::string>& class_names,
                                                      const builtin_interfaces::msg::Time& stamp,
                                                      const AnnotationStyle& style) {
  foxglove_msgs::msg::ImageAnnotations message;
  // 设置顶层 timestamp 后，单条标注自身的 timestamp 会被忽略
  message.timestamp = stamp;

  for (std::size_t i = 0; i < detections.size(); ++i) {
    const SegmentDetection& detection = detections[i];
    const int track_id = i < track_ids.size() ? track_ids[i] : -1;
       
    if (!detection.mask.empty()) {
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(detection.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto& contour : contours) {
        if (contour.size() >= 3) {
          message.points.push_back(
              MakePolygon(contour, style.mask_outline_color, style.mask_fill_color, stamp, style.line_thickness));
        }
      }
    }

    const std::vector<cv::Point> box_points{
        cv::Point(static_cast<int>(detection.bbox.x1), static_cast<int>(detection.bbox.y1)),
        cv::Point(static_cast<int>(detection.bbox.x2), static_cast<int>(detection.bbox.y1)),
        cv::Point(static_cast<int>(detection.bbox.x2), static_cast<int>(detection.bbox.y2)),
        cv::Point(static_cast<int>(detection.bbox.x1), static_cast<int>(detection.bbox.y2))};
    message.points.push_back(
        MakePolygon(box_points, style.box_color, {0.0, 0.0, 0.0, 0.0}, stamp, style.line_thickness));

    std::string label = detection.class_id >= 0 && static_cast<std::size_t>(detection.class_id) < class_names.size()
                            ? class_names[static_cast<std::size_t>(detection.class_id)]
                            : std::to_string(detection.class_id);
    if (track_id >= 0) {
      label = "#" + std::to_string(track_id) + " " + label;
    }
    label += cv::format(" %.2f", static_cast<double>(detection.confidence));

    foxglove_msgs::msg::TextAnnotation text;
    text.timestamp = stamp;
    text.position.x = detection.bbox.x1;
    text.position.y = std::max(detection.bbox.y1 - style.font_size, 0.0);
    text.text = label;
    text.font_size = style.font_size;
    text.text_color = MakeColor(style.text_color);
    text.background_color = MakeColor(style.text_background_color);
    message.texts.push_back(text);
  }

  return message;
}

sensor_msgs::msg::PointCloud2 BuildPointCloud2(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                               const std_msgs::msg::Header& header,
                                               const std::array<float, 3>& rgb, int stride) {
  const auto step = static_cast<std::size_t>(stride > 0 ? stride : 1);

  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.height = 1;
  message.is_bigendian = false;
  message.point_step = 16;  // x/y/z/rgb 各 4 字节
  message.is_dense = true;

  const char* const names[4] = {"x", "y", "z", "rgb"};
  message.fields.resize(4);
  for (std::size_t i = 0; i < 4; ++i) {
    message.fields[i].name = names[i];
    message.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
    message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[i].count = 1;
  }

  message.width = static_cast<std::uint32_t>((cloud.size() + step - 1) / step);
  message.row_step = message.point_step * message.width;
  message.data.resize(message.row_step);

  const float packed_rgb = PackRgb(rgb);
  auto* out = reinterpret_cast<float*>(message.data.data());
  std::uint32_t written = 0;
  for (std::size_t i = 0; i < cloud.size(); i += step) {
    const pcl::PointXYZ& point = cloud[i];
    *out++ = point.x;
    *out++ = point.y;
    *out++ = point.z;
    *out++ = packed_rgb;
    ++written;
  }

  if (written != message.width) {
    message.width = written;
    message.row_step = message.point_step * message.width;
    message.data.resize(message.row_step);
  }
  return message;
}

void DebugCloudPool::Publish(const PoseDebugClouds& clouds, int detection_index, const std_msgs::msg::Header& header) {
  const std::string base = topic_prefix_ + "/det" + std::to_string(detection_index) + "/";
  PublishOne(base + "scene", clouds.scene_points, header, kSceneRgb);
  PublishOne(base + "model_pca", clouds.model_pca, header, kModelPcaRgb);
  PublishOne(base + "model_icp", clouds.model_icp, header, kModelIcpRgb);
}

void DebugCloudPool::PublishOne(const std::string& topic, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                const std_msgs::msg::Header& header, const std::array<float, 3>& rgb) {
  if (cloud.empty()) {
    return;
  }
  pool_.Publish(topic, BuildPointCloud2(cloud, header, rgb, stride_));
}

}  // namespace amorph::vision
