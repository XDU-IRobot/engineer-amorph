// 用真实 Sort3DTracker 验证参数改动：目标匀速移动后急停，测量加噪声，
// 观察 observe 输出的过冲与静止抖动。
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <random>
#include <vector>

#include "amorph_vision_cpp/sort3d_tracker.hpp"

namespace amorph::vision {
namespace {

struct ScenarioResult {
  double overshoot_mm = 0.0;
  double static_jitter_mm = 0.0;
};

/// 匀速 20 帧 -> 急停 60 帧，返回过冲（相对停止点）与静止抖动。
ScenarioResult RunStopScenario(double process_var, double measurement_var, double sigma, unsigned seed) {
  Sort3DTracker tracker(/*max_age=*/15, /*min_hits=*/1, /*mahalanobis_thresh=*/15.0, /*iou_weight=*/0.4,
                        process_var, measurement_var);
  std::mt19937 generator(seed);
  std::normal_distribution<double> noise(0.0, sigma);

  constexpr double kDt = 1.0 / 30.0;
  constexpr double kVelocity = 0.6;  // m/s
  constexpr int kMovingFrames = 20;
  constexpr int kStaticFrames = 60;

  double true_position = 0.0;
  double stop_position = 0.0;
  std::vector<int> removed;
  ScenarioResult result;
  std::vector<double> static_errors;

  for (int frame = 0; frame < kMovingFrames + kStaticFrames; ++frame) {
    const double velocity = frame < kMovingFrames ? kVelocity : 0.0;
    if (frame == kMovingFrames) {
      stop_position = true_position;
    }
    true_position += velocity * kDt;

    Detection detection;
    detection.position = Eigen::Vector3d(true_position + noise(generator), 0.0, 0.0);
    detection.bbox = BoundingBox{100.0, 100.0, 200.0, 200.0};

    const auto assignments = tracker.Update({detection}, kDt, &removed);
    if (assignments.empty()) {
      continue;
    }
    const double filtered = assignments.front().filtered_position.x();
    const double error = filtered - true_position;

    if (frame < kMovingFrames) {
      continue;
    }
    if (frame < kMovingFrames + 20) {
      result.overshoot_mm = std::max(result.overshoot_mm, (filtered - stop_position) * 1000.0);
    } else {
      static_errors.push_back(error);
    }
  }

  double mean = 0.0;
  for (const double value : static_errors) {
    mean += value;
  }
  mean /= static_cast<double>(static_errors.size());
  double variance = 0.0;
  for (const double value : static_errors) {
    variance += (value - mean) * (value - mean);
  }
  result.static_jitter_mm = std::sqrt(variance / static_cast<double>(static_errors.size())) * 1000.0;
  return result;
}

TEST(Sort3DTracker, TunedNoiseReducesStopOvershoot) {
  // 默认参数应显著优于旧默认值 (0.5, 0.01)
  const ScenarioResult old_params = RunStopScenario(0.5, 0.01, 0.003, 7);
  const ScenarioResult new_params = RunStopScenario(5.0, 1e-3, 0.003, 7);

  std::printf("  旧 (q=0.5, r=1e-2): 过冲 %.1fmm 抖动 %.2fmm\n", old_params.overshoot_mm,
              old_params.static_jitter_mm);
  std::printf("  新 (q=5.0, r=1e-3): 过冲 %.1fmm 抖动 %.2fmm\n", new_params.overshoot_mm,
              new_params.static_jitter_mm);

  // 过冲的理论下限是一帧的位移 v*dt = 20mm，新参数应接近它
  EXPECT_LT(new_params.overshoot_mm, 35.0);
  EXPECT_LT(new_params.overshoot_mm, old_params.overshoot_mm * 0.6);
  EXPECT_LT(new_params.static_jitter_mm, 3.0);
  EXPECT_LT(new_params.static_jitter_mm, old_params.static_jitter_mm * 0.5);
}

}  // namespace
}  // namespace amorph::vision
