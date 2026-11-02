#include "amorph_vision_cpp/eu_locator_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <omp.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <utility>

#include "amorph_vision_cpp/icp_refine.hpp"
#include "amorph_vision_cpp/visualization.hpp"

namespace amorph::vision {
namespace {

/// 同步队列深度与前后的时间容差（秒），与参考实现一致。
constexpr int kSyncQueueSize = 5;
constexpr double kSyncSlopSec = 0.05;
/// 帧间隔的合法范围，避免 bag 回放或时间戳跳变使卡尔曼预测发散。
constexpr double kMinDeltaSec = 1.0 / 120.0;
constexpr double kMaxDeltaSec = 0.5;

std::string GetShareDirectory() {
  try {
    return ament_index_cpp::get_package_share_directory("amorph_vision_cpp");
  } catch (const std::exception&) {
    return {};
  }
}

/// 资产装在 share/<pkg>/resource/ 下（见 CMakeLists 的 install 规则）。
constexpr char kResourceSubdir[] = "resource";

/// 解析资产路径：参数留空时回退到 share/<pkg>/resource/<filename>。
///
/// 注意参数为空并不等于“没设置”——launch 会把 engine_path:'' 显式传进来，
/// 此时 declare_parameter 拿到的是空串而不是默认值，所以必须在这里统一判空。
std::string ResolveAssetPath(const std::string& param_value, const std::string& share_dir,
                             const std::string& filename) {
  if (!param_value.empty()) {
    return param_value;
  }
  if (share_dir.empty()) {
    return {};
  }
  return share_dir + "/" + kResourceSubdir + "/" + filename;
}

}  // namespace

EuLocatorNode::EuLocatorNode() : rclcpp::Node("eu_locator_node"), params_(*this) {
  params_.Init();
  const RosParams& params = params_.data();
  const std::string share_dir = GetShareDirectory();

  const std::string engine_path = ResolveAssetPath(params.model.engine_path, share_dir, "best.engine");
  const std::string ply_path = ResolveAssetPath(params.model.ply_path, share_dir, "energy_unit.ply");

  tracker_ = std::make_unique<Sort3DTracker>(params.tracking.max_age, params.tracking.min_hits,
                                             params.tracking.mahalanobis_thresh, params.tracking.iou_weight,
                                             params.tracking.process_var, params.tracking.measurement_var);

  if (engine_path.empty()) {
    throw std::runtime_error("找不到 engine：engine_path 为空且包 share 目录不可用");
  }
  if (!std::filesystem::exists(engine_path)) {
    throw std::runtime_error("engine 文件不存在: " + engine_path);
  }
  RCLCPP_INFO(get_logger(), "Loading TensorRT engine: %s", engine_path.c_str());
  yolo_ = std::make_unique<YoloSegmenter>(engine_path, cv::Size(params.model.input_width, params.model.input_height));

  if (ply_path.empty() || !std::filesystem::exists(ply_path)) {
    throw std::runtime_error("PLY 模型不存在: " + (ply_path.empty() ? std::string("<未解析>") : ply_path));
  }
  RCLCPP_INFO(get_logger(), "Loading EU model: %s", ply_path.c_str());
  eu_model_ = std::make_unique<EuModel>(ply_path);
  RCLCPP_INFO(get_logger(), "Model sampled: %zu points, max_extent=%.4fm", eu_model_->size(), eu_model_->max_extent());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // 高频状态量，best effort 即可；用 reliable 时订阅端跟不上会阻塞 publish，反过来拖慢整个图像回调
  detection_pool_ = std::make_unique<PublisherPool<vision_msgs::msg::Detection3DArray>>(*this, rclcpp::SensorDataQoS());
  annotation_pool_ =
      std::make_unique<PublisherPool<foxglove_msgs::msg::ImageAnnotations>>(*this, rclcpp::SensorDataQoS());
  if (params.publish_debug_clouds) {
    debug_cloud_pool_ = std::make_unique<DebugCloudPool>(*this, "debug_clouds", params.debug_cloud_stride);
    RCLCPP_INFO(get_logger(), "调试点云已开启：debug_clouds/det<i>/{scene,model_pca,model_icp}，抽稀步长 %d",
                params.debug_cloud_stride > 0 ? params.debug_cloud_stride : 1);
  }

  // 相机驱动可能以 RELIABLE 或 BEST_EFFORT 发布，SensorDataQoS 可兼容两者
  rgb_sub_.subscribe(this, params.topics.rgb, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  depth_sub_.subscribe(this, params.topics.depth, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(kSyncQueueSize), rgb_sub_, depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(kSyncSlopSec));
  sync_->registerCallback(std::bind(&EuLocatorNode::OnFrame, this, std::placeholders::_1, std::placeholders::_2));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      params.topics.camera_info, 10, std::bind(&EuLocatorNode::OnCameraInfo, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "node ready (rgb=%s depth=%s info=%s)", params.topics.rgb.c_str(),
              params.topics.depth.c_str(), params.topics.camera_info.c_str());
}

void EuLocatorNode::OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message) {
  if (has_intrinsics_) {
    return;
  }
  intrinsics_.fx = message->k[0];
  intrinsics_.fy = message->k[4];
  intrinsics_.cx = message->k[2];
  intrinsics_.cy = message->k[5];
  if (intrinsics_.fx <= 0.0 || intrinsics_.fy <= 0.0) {
    RCLCPP_WARN(get_logger(), "DISCARDING INVALID CAMERA INFO");
    return;
  }
  has_intrinsics_ = true;
  RCLCPP_INFO(get_logger(), "Intrinsics ready: fx=%.1f fy=%.1f", intrinsics_.fx, intrinsics_.fy);

  // 收到camera info，构造PcaLocator
  if (locator_ != nullptr || !has_intrinsics_) {
    return;
  }
  locator_ = std::make_unique<PcaLocator>(intrinsics_, params_.data().pca.erosion_kernel_size,
                                          params_.data().pca.erosion_iterations);
}

void EuLocatorNode::OnFrame(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                            const sensor_msgs::msg::Image::ConstSharedPtr& depth_message) {
  if (locator_ == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera_info...");
    return;
  }

  const auto frame_start = std::chrono::steady_clock::now();

  const cv::Mat color = cv_bridge::toCvShare(rgb, "bgr8")->image;

  const auto to_depth_millimeters = [](const sensor_msgs::msg::Image::ConstSharedPtr& message) -> cv::Mat {
    if (message->encoding == "16UC1") {
      const cv_bridge::CvImageConstPtr bridge = cv_bridge::toCvShare(message, "16UC1");
      cv::Mat depth;
      bridge->image.convertTo(depth, CV_32F);
      return depth;
    }
    if (message->encoding == "32FC1") {
      const cv_bridge::CvImageConstPtr bridge = cv_bridge::toCvShare(message, "32FC1");
      cv::Mat depth;
      bridge->image.convertTo(depth, CV_32F, 1000.0);
      return depth;
    }
    return {};
  };

  const cv::Mat depth = to_depth_millimeters(depth_message);
  if (depth.empty()) {
    RCLCPP_ERROR(get_logger(), "Unsupported depth encoding: %s", depth_message->encoding.c_str());
    return;
  }

  // 计算帧间隔dt，给tracker里的卡尔曼滤波器使用
  const double stamp_sec =
      static_cast<double>(rgb->header.stamp.sec) + static_cast<double>(rgb->header.stamp.nanosec) * 1e-9;
  double dt = 1.0 / 30.0;
  if (last_stamp_sec_ >= 0.0) {
    dt = std::clamp(stamp_sec - last_stamp_sec_, kMinDeltaSec, kMaxDeltaSec);
  }
  last_stamp_sec_ = stamp_sec;

  const auto yolo_start = std::chrono::steady_clock::now();
  std::vector<SegmentDetection> segmentations = yolo_->Detect(color, static_cast<float>(params_.data().model.conf_threshold));
  const auto yolo_end = std::chrono::steady_clock::now();

  const bool publish_debug_clouds = params_.data().publish_debug_clouds;

  // 拿到能量单元的mask，开始位姿估计（PCA估主轴方向+降自由度ICP精配准）
  //
  // PcaLocator::ProcessMask 与 EuModel::SampleVisible 都是 const 只读，RefinePose 把 ICP
  // refiner、kd-tree、体素网格、法向估计全部构造在函数内部，因此每个物体instance的估计可以并行
  //
  // ICP比较费时，在电脑上处理一次大约20ms，而在场上一般会同时看见5个以上的能量单元
  // 所以把这5、6个能量单元的算法并行起来可以获得非常大的效率收益

  // detections 的顺序必须与 segmentations 一致
  std::vector<int> annotation_ids(segmentations.size(), -1);

  /// 估计结果 + 调试点云，两者同生命周期，压实后仍一一对应
  struct EstimateResult {
    Detection detection;
    PoseDebugClouds clouds;
  };
  std::vector<std::optional<EstimateResult>> estimated(segmentations.size());
  std::vector<char> failed(segmentations.size(), 0);

  const auto target_count = static_cast<std::ptrdiff_t>(segmentations.size());
  // 一般不会输出多于8个目标
  const int thread_count =
      static_cast<int>(std::min<std::ptrdiff_t>(target_count, static_cast<std::ptrdiff_t>(omp_get_max_threads())));

#pragma omp parallel for schedule(dynamic, 1) if (target_count > 1) num_threads(thread_count)
  for (std::ptrdiff_t i = 0; i < target_count; ++i) {
    const auto index = static_cast<std::size_t>(i);
    const SegmentDetection& segmentation = segmentations[index];
    try {  // 防止worker线程里的异常触发std::terminate
      const auto estimate = locator_->ProcessMask(segmentation.mask, segmentation.bbox, depth);
      if (!estimate.has_value()) {
        continue;
      }

      Pose pose = estimate->pose;
      // icp refine
      const auto target_pts = ExtractMaskedPoints(depth, estimate->binary_mask, intrinsics_);  // 深度相机点云抠出来的点
      const auto src_pts = eu_model_->SampleVisible(pose);  // ply模型采样出来的点
      const Pose refined = RefinePose(*src_pts, *target_pts, pose);

      EstimateResult result;
      result.detection.position = refined.position;
      result.detection.orientation = pose.orientation;
      result.detection.bbox = segmentation.bbox;
      result.detection.confidence = segmentation.confidence;
      result.detection.class_id = segmentation.class_id;
      result.detection.mask = segmentation.mask;
      result.detection.annotation_index = segmentation.annotation_index;

      if (publish_debug_clouds) {
        result.clouds.scene_points = *target_pts;
        result.clouds.model_pca = *src_pts;
        // ICP 只优化平移，旋转冻结；按精配准后的位姿重新采样，忠实反映最终摆放
        Pose icp_pose = pose;
        icp_pose.position = refined.position;
        result.clouds.model_icp = *eu_model_->SampleVisible(icp_pose);
      }

      estimated[index] = std::move(result);
    } catch (const std::exception&) {
      // 触发异常视为失败
      failed[index] = 1;
    }
  }

  std::vector<Detection> detections;
  std::vector<PoseDebugClouds> frame_clouds;  // 与 detections 同序
  detections.reserve(segmentations.size());
  if (publish_debug_clouds) {
    frame_clouds.reserve(segmentations.size());
  }
  for (auto& item : estimated) {
    if (!item.has_value()) {
      continue;
    }
    detections.push_back(std::move(item->detection));
    if (publish_debug_clouds) {
      frame_clouds.push_back(std::move(item->clouds));
    }
  }
  if (const auto failed_count = std::count(failed.begin(), failed.end(), 1); failed_count > 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%ld 个目标的位姿估计抛出异常，已跳过",
                         static_cast<long>(failed_count));
  }
  const auto estimate_end = std::chrono::steady_clock::now();

  std::vector<int> removed_ids;
  // 每帧同步一次，这样现场 ros2 param set 调整噪声参数可以立即生效
  tracker_->SetNoiseParams(params_.data().tracking.process_var, params_.data().tracking.measurement_var);
  const std::vector<TrackAssignment> assignments = tracker_->Update(detections, dt, &removed_ids);
  for (const int track_id : removed_ids) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "track %d dropped", track_id);
  }

  for (const TrackAssignment& assignment : assignments) {
    const Detection& detection = detections[static_cast<std::size_t>(assignment.detection_index)];
    if (detection.annotation_index >= 0) {
      annotation_ids[static_cast<std::size_t>(detection.annotation_index)] = assignment.track_id;
    }

    const std::string frame = "energy_unit_" + std::to_string(assignment.track_id);
    // 原始测量：单帧直出，抖动明显
    PublishTransform(rgb->header, frame, detection.position, detection.orientation);
    // 卡尔曼滤波后的位置，方向仍用本帧测量（KF 不估姿态）
    PublishTransform(rgb->header, frame + "_observe", assignment.filtered_position, detection.orientation);
  }

  detection_pool_->Publish(params_.data().topics.detections, BuildDetectionArray(rgb->header, assignments, detections));

  annotation_pool_->Publish(
      "yolo_annotation",
      BuildAnnotations(segmentations, annotation_ids, params_.data().model.class_names, rgb->header.stamp));

  // 调试点云：三组叠看即可判断是掩膜、粗定位还是精配准的问题。
  // 只按 detection 下标命名，不跟随 tracker 分配的 track id —— 排查阶段关心的是
  // 本帧每一个检测自身的估计质量，与它跨帧的身份无关。
  if (publish_debug_clouds && debug_cloud_pool_) {
    sensor_msgs::msg::Image::_header_type header;
    header.stamp = rgb->header.stamp;
    header.frame_id = rgb->header.frame_id;
    for (std::size_t i = 0; i < detections.size(); ++i) {
      debug_cloud_pool_->Publish(frame_clouds[i], detections[i].annotation_index, header);
    }
  }

  if (params_.data().log_timing) {
    const auto frame_end = std::chrono::steady_clock::now();
    const auto to_ms = [](const auto& duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
    RCLCPP_INFO(get_logger(), "YOLO %.2fms | PCA+ICP %.2fms | total %.2fms | %zu detected, %zu published",
                to_ms(yolo_end - yolo_start), to_ms(estimate_end - yolo_end), to_ms(frame_end - frame_start),
                detections.size(), assignments.size());
  } else {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Frame processed: %zu published, %zu detected",
                          assignments.size(), detections.size());
  }
}

}  // namespace amorph::vision
