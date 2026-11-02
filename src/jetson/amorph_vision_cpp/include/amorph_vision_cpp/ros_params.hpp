#pragma once

#include <string>
#include <vector>

namespace amorph::vision {

/// 节点参数。字段名即 ROS 参数名，嵌套结构体会展开成 `a.b`（如 `model.conf_threshold`）。
///
/// 默认值直接写在结构体里：ParamsManager 用 declare_parameter 声明时以它们为默认值，
/// launch / YAML 的覆盖值优先。
///
/// 注意：订阅话题、tracker、发布器都在构造期按参数建好，所以改这些参数不会立即生效；
/// 只按帧读取的参数（conf_threshold、log_timing 等）可以运行时动态调整。
struct RosParams {
  struct Topics {
    std::string rgb = "/camera/color/image_raw";
    std::string depth = "/camera/depth/image_raw";
    std::string camera_info = "/camera/color/camera_info";
    std::string detections = "eu_detections";
  } topics;

  struct Model {
    /// 留空则回退到 share/<pkg>/resource/ 下的同名文件
    std::string engine_path;
    std::string ply_path;
    int input_width = 640;
    int input_height = 640;
    double conf_threshold = 0.85;
    std::vector<std::string> class_names{"energy_unit"};
  } model;

  struct Pca {
    int erosion_kernel_size = 5;
    int erosion_iterations = 1;
  } pca;

  struct Tracking {
    int max_age = 15;
    int min_hits = 3;
    double mahalanobis_thresh = 15.0;
    double iou_weight = 0.4;
    /// 卡尔曼过程噪声（白噪声加速度的功率谱密度）。越大速度越跟随测量，
    /// 急停后的过冲越小，代价是静止时抽动更多。
    double process_var = 5.0;
    /// 测量噪声方差（m^2）。影响平滑强度，也决定马氏门限的物理容差
    /// （半径约 sqrt(mahalanobis_thresh * measurement_var)）。
    double measurement_var = 1e-3;
  } tracking;

  bool log_timing = false;

  /// 把 PCA / ICP 的中间点云发布到 debug_clouds/det<i>/{scene,model_pca,model_icp}
  bool publish_debug_clouds = false;
  int debug_cloud_stride = 4;
};

}  // namespace amorph::vision
