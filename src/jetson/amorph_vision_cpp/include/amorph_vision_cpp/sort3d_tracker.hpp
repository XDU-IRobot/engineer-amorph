// 3D SORT 跟踪：卡尔曼滤波 + 匈牙利匹配，为多目标分配稳定 track_id。
//
// 与 2D 版本的区别：关联用 PCA + ICP 得到的 3D 位置，并把 3D 位置的马氏距离
// 与 2D 框的 IoU 一起作为代价。目标短暂跟丢时轨迹保留 max_age 帧并继续用
// 卡尔曼预测位置，因此重新出现时能匹配回原来的 id。
#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

#include "amorph_vision_cpp/types.hpp"

namespace amorph::vision {

class Sort3DTracker {
 public:
  explicit Sort3DTracker(int max_age = 15, int min_hits = 1, double mahalanobis_thresh = 15.0, double iou_weight = 0.4,
                         double process_var = 5.0, double measurement_var = 1e-3)
      : max_age_(max_age),
        min_hits_(min_hits),
        mahalanobis_thresh_(mahalanobis_thresh),
        iou_weight_(iou_weight),
        process_var_(process_var),
        measurement_var_(measurement_var) {
    for (int id = 0; id < kMaxTracks; ++id) {
      available_ids_.push_back(id);
    }
  }

  void Reset();

  /// 运行时改 KF 噪声参数，便于现场调参而不必重启节点。
  void SetNoiseParams(double process_var, double measurement_var) {
    process_var_ = process_var;
    measurement_var_ = measurement_var;
  }

  /// 用当前帧检测更新轨迹。
  /// @param detections 检测列表，需要 position / bbox 字段。
  /// @param dt         帧间隔（秒）。
  /// @param removed_ids 输出：被移除的 track id，调用方应清理相关状态。
  std::vector<TrackAssignment> Update(const std::vector<Detection>& detections, double dt,
                                      std::vector<int>* removed_ids);

 private:
  /// 状态为 [x, y, z, vx, vy, vz] 的匀速模型轨迹。
  struct KalmanTrack3D {
    int track_id = 0;
    Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    BoundingBox last_bbox;
    int hits = 1;
    int age = 1;
    int missed = 0;

    void Predict(double dt, double process_var);
    void Update(const Eigen::Vector3d& measurement, const BoundingBox& bbox, double measurement_var);
    double MahalanobisSquared(const Eigen::Vector3d& measurement, double measurement_var) const;
  };

  struct CreateResult {
    int track_id = -1;
    std::optional<int> evicted_id;
  };

  CreateResult CreateTrack(const Eigen::Vector3d& position, const BoundingBox& bbox);
  void MarkMostRecentlyUsed(int track_id);
  std::vector<int> PruneDeadTracks();
  std::vector<int> PruneSpecificTracks(const std::vector<int>& track_ids);
  void ReleaseTrackId(int track_id);
  std::vector<std::vector<double>> BuildCostMatrix(const std::vector<int>& track_ids,
                                                   const std::vector<Eigen::Vector3d>& positions,
                                                   const std::vector<BoundingBox>& boxes) const;

  static constexpr int kMaxTracks = 8;

  int max_age_;
  int min_hits_;
  double mahalanobis_thresh_;
  double iou_weight_;
  double process_var_;
  double measurement_var_;

  std::vector<int> available_ids_;  ///< 空闲 id 池
  std::vector<int> lru_order_;      ///< 队首最久未使用
  std::map<int, KalmanTrack3D> tracks_;
};

}  // namespace amorph::vision
