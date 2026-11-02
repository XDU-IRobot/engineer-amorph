#include "amorph_vision_cpp/yolo_segmenter.hpp"

#include <opencv2/imgproc.hpp>
#include <trtyolo.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace amorph::vision {
namespace {

/// 把单个检测的掩膜铺回整幅图像。
///
/// 关键点：`ROIAlign_TRT` 的输出**已经按该检测的框裁剪过**，是一张 bbox 局部
/// 的 160×160 掩膜（不是整图低分辨率掩膜）。所以流程是
/// resize 到框的像素尺寸 → 二值化 → 按框左上角偏移粘贴，而不是把整幅图缩放。
/// 这也意味着 letterbox 的填充边已经由插件配合框坐标处理掉了。
cv::Mat MaskToFullImage(const trtyolo::Mask& mask, const BoundingBox& bbox, const cv::Size& image) {
  cv::Mat full = cv::Mat::zeros(image, CV_8U);
  if (mask.data.empty() || mask.width <= 0 || mask.height <= 0) {
    return full;
  }

  // 框坐标由 TensorRT-YOLO 的后处理映射回原图坐标系，这里是闭区间像素边界
  const int bx1 = static_cast<int>(std::lround(bbox.x1));
  const int by1 = static_cast<int>(std::lround(bbox.y1));
  const int bx2 = static_cast<int>(std::lround(bbox.x2));
  const int by2 = static_cast<int>(std::lround(bbox.y2));

  const int box_w = std::max(bx2 - bx1 + 1, 1);
  const int box_h = std::max(by2 - by1 + 1, 1);

  // 只读包装 TensorRT-YOLO 的缓冲区，不做像素拷贝
  const cv::Mat local_mask(mask.height, mask.width, CV_32FC1, const_cast<float*>(mask.data.data()));

  cv::Mat resized;
  cv::resize(local_mask, resized, cv::Size(box_w, box_h), 0.0, 0.0, cv::INTER_LINEAR);

  cv::Mat binary;
  cv::threshold(resized, binary, 0.5, 255.0, cv::THRESH_BINARY);
  binary.convertTo(binary, CV_8U);

  // 框可能部分越界，需要在目标/源两侧同时裁剪
  const int dst_x1 = std::max(0, bx1);
  const int dst_y1 = std::max(0, by1);
  const int dst_x2 = std::min(image.width, bx2 + 1);
  const int dst_y2 = std::min(image.height, by2 + 1);
  const int src_x = std::max(0, -bx1);
  const int src_y = std::max(0, -by1);

  int copy_w = dst_x2 - dst_x1;
  int copy_h = dst_y2 - dst_y1;
  if (copy_w <= 0 || copy_h <= 0) {
    return full;
  }
  copy_w = std::min(copy_w, binary.cols - src_x);
  copy_h = std::min(copy_h, binary.rows - src_y);
  if (copy_w <= 0 || copy_h <= 0) {
    return full;
  }

  binary(cv::Rect(src_x, src_y, copy_w, copy_h))
      .copyTo(full(cv::Rect(dst_x1, dst_y1, copy_w, copy_h)));
  return full;
}

}  // namespace

YoloSegmenter::YoloSegmenter(const std::string& engine_path, cv::Size model_input_size)
    : model_input_size_(model_input_size) {
  trtyolo::InferOption option;
  option.enableSwapRB();  // 输入为 OpenCV 的 BGR，模型需要 RGB
  model_ = std::make_unique<trtyolo::SegmentModel>(engine_path, option);
}

YoloSegmenter::~YoloSegmenter() = default;

std::vector<SegmentDetection> YoloSegmenter::Detect(const cv::Mat& bgr, float conf_threshold) {
  std::vector<SegmentDetection> detections;
  if (bgr.empty()) {
    return detections;
  }

  // trtyolo::Image 不做像素拷贝，要求行连续
  const cv::Mat input = bgr.isContinuous() ? bgr : bgr.clone();
  trtyolo::Image image(input.data, input.cols, input.rows);

  const trtyolo::SegmentRes result = model_->predict(image);

  for (int i = 0; i < result.num; ++i) {
    if (result.scores[i] < conf_threshold) {
      continue;
    }
    SegmentDetection detection;
    const auto xyxy = result.boxes[i].xyxy();
    detection.bbox.x1 = xyxy[0];
    detection.bbox.y1 = xyxy[1];
    detection.bbox.x2 = xyxy[2];
    detection.bbox.y2 = xyxy[3];
    detection.confidence = result.scores[i];
    detection.class_id = result.classes[i];
    detection.annotation_index = i;
    detection.mask = i < static_cast<int>(result.masks.size())
                         ? MaskToFullImage(result.masks[i], detection.bbox, bgr.size())
                         : cv::Mat::zeros(bgr.size(), CV_8U);
    detections.push_back(std::move(detection));
  }

  return detections;
}

}  // namespace amorph::vision
