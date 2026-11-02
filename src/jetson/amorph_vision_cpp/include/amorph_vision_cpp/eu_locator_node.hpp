// 能量单元定位节点：RGB + Depth → YOLO-seg → PCA → ICP → 3D SORT → TF。
#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <foxglove_msgs/msg/image_annotations.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <memory>
#include <string>
#include <vector>

#include "amorph_vision_cpp/eu_model.hpp"
#include "amorph_vision_cpp/pca_locator.hpp"
#include "amorph_vision_cpp/publisher_pool.hpp"
#include "amorph_vision_cpp/ros_params.hpp"
#include "amorph_vision_cpp/ros_params_manager.hpp"
#include "amorph_vision_cpp/sort3d_tracker.hpp"
#include "amorph_vision_cpp/types.hpp"
#include "amorph_vision_cpp/visualization.hpp"
#include "amorph_vision_cpp/yolo_segmenter.hpp"

namespace amorph::vision {

class EuLocatorNode : public rclcpp::Node {
 public:
  EuLocatorNode();

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  // message callbacks
  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message);
  void OnFrame(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
               const sensor_msgs::msg::Image::ConstSharedPtr& depth);

  /// 广播一条 TF。位置与姿态分开传入：滤波后的位置配当帧测得的姿态。
  void PublishTransform(const std_msgs::msg::Header& header, const std::string& child_frame_id,
                        const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    tf_broadcaster_->sendTransform(transform);
  }

  /// 把本帧跟踪结果打包成 Detection3DArray。位姿用滤波后的位置配本帧测得的姿态，与 TF 的 _observe 帧一致。
  vision_msgs::msg::Detection3DArray BuildDetectionArray(const std_msgs::msg::Header& header,
                                                         const std::vector<TrackAssignment>& assignments,
                                                         const std::vector<Detection>& detections) const {
    vision_msgs::msg::Detection3DArray array;
    array.header = header;
    array.detections.reserve(assignments.size());
    // 能量单元是回转体，包围盒用模型外接球直径，方向无意义，只供 RViz 显示大小
    const double extent = 2.0 * eu_model_->max_extent();
    const std::vector<std::string>& class_names = params_.data().model.class_names;
    for (const TrackAssignment& assignment : assignments) {
      const Detection& detection = detections[static_cast<std::size_t>(assignment.detection_index)];

      geometry_msgs::msg::Pose pose;
      pose.position.x = assignment.filtered_position.x();
      pose.position.y = assignment.filtered_position.y();
      pose.position.z = assignment.filtered_position.z();
      pose.orientation.x = detection.orientation.x();
      pose.orientation.y = detection.orientation.y();
      pose.orientation.z = detection.orientation.z();
      pose.orientation.w = detection.orientation.w();

      vision_msgs::msg::Detection3D detected;
      detected.header = header;
      detected.id = std::to_string(assignment.track_id);
      detected.bbox.center = pose;
      detected.bbox.size.x = detected.bbox.size.y = detected.bbox.size.z = extent;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id =
          detection.class_id >= 0 && static_cast<std::size_t>(detection.class_id) < class_names.size()
              ? class_names[static_cast<std::size_t>(detection.class_id)]
              : std::to_string(detection.class_id);
      hypothesis.hypothesis.score = detection.confidence;
      hypothesis.pose.pose = pose;
      detected.results.push_back(std::move(hypothesis));

      array.detections.push_back(std::move(detected));
    }
    return array;
  }

  // 参数（Init() 之后用 params_.data() 取值）
  ParamsManager<RosParams> params_;

  // 算法组件
  std::unique_ptr<YoloSegmenter> yolo_;
  std::unique_ptr<EuModel> eu_model_;
  std::unique_ptr<PcaLocator> locator_;
  std::unique_ptr<Sort3DTracker> tracker_;

  // ROS接口
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::unique_ptr<PublisherPool<foxglove_msgs::msg::ImageAnnotations>> annotation_pool_;
  std::unique_ptr<PublisherPool<vision_msgs::msg::Detection3DArray>> detection_pool_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<DebugCloudPool> debug_cloud_pool_;

  // 状态
  bool has_intrinsics_ = false;
  Intrinsics intrinsics_;
  double last_stamp_sec_ = -1.0;
};

}  // namespace amorph::vision
