// TensorRT-YOLO 实例分割封装：engine → 全分辨率掩膜 + 检测框。
#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

#include "amorph_vision_cpp/types.hpp"

namespace trtyolo {
class SegmentModel;
}

namespace amorph::vision {

/// 单个分割检测（尚未估位姿）。
struct SegmentDetection {
  BoundingBox bbox;
  float confidence = 0.0F;
  int class_id = 0;
  /// 全分辨率二值掩膜，CV_8UC1，前景 255。
  cv::Mat mask;
  /// 该检测在 YOLO 原始输出中的下标。
  int annotation_index = -1;
};

class YoloSegmenter {
 public:
  /// @param engine_path       TensorRT engine（由 trtyolo-export + trtexec 生成）
  /// @param model_input_size  engine 的静态输入尺寸，用于把掩膜从 letterbox 空间映射回原图
  YoloSegmenter(const std::string& engine_path, cv::Size model_input_size);
  ~YoloSegmenter();

  YoloSegmenter(const YoloSegmenter&) = delete;
  YoloSegmenter& operator=(const YoloSegmenter&) = delete;

  /// 对一张 BGR 图像做实例分割，并按阈值过滤。
  std::vector<SegmentDetection> Detect(const cv::Mat& bgr, float conf_threshold);

 private:
  std::unique_ptr<trtyolo::SegmentModel> model_;
  cv::Size model_input_size_;
};

}  // namespace amorph::vision
