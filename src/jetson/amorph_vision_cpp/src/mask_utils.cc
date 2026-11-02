#include "amorph_vision_cpp/mask_utils.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

#include "amorph_vision_cpp/numeric_utils.hpp"

namespace amorph::vision::mask {

cv::Mat Erode(const cv::Mat& mask, int kernel_size, int iterations) {
  if (mask.empty() || kernel_size <= 1 || iterations <= 0) {
    return mask.clone();
  }
  const cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8U);
  cv::Mat out;
  cv::erode(mask, out, kernel, cv::Point(-1, -1), iterations);
  return out;
}

cv::Mat LargestComponent(const cv::Mat& binary) {
  if (binary.empty()) {
    return {};
  }
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int num = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
  if (num <= 1) {
    return {};
  }

  int best_label = 1;
  int best_area = stats.at<int>(1, cv::CC_STAT_AREA);
  for (int label = 2; label < num; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area > best_area) {
      best_area = area;
      best_label = label;
    }
  }

  cv::Mat out = (labels == best_label);
  return out;  // 0/1 的 CV_8U
}

bool MedianDepth(const cv::Mat& depth_mm, const cv::Mat& mask, double* median_mm) {
  std::vector<float> values;
  values.reserve(static_cast<std::size_t>(cv::countNonZero(mask)));
  for (int y = 0; y < mask.rows; ++y) {
    const auto* mask_row = mask.ptr<std::uint8_t>(y);
    const auto* depth_row = depth_mm.ptr<float>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (mask_row[x] == 0) {
        continue;
      }
      if (depth_row[x] > 0.0F) {
        values.push_back(depth_row[x]);
      }
    }
  }
  if (values.empty()) {
    return false;
  }
  *median_mm = Median(std::move(values));
  return true;
}

cv::Mat FilterByDepth(const cv::Mat& depth_mm, const cv::Mat& mask, double median_mm, double threshold_m) {
  cv::Mat out = cv::Mat::zeros(mask.size(), CV_8U);
  for (int y = 0; y < mask.rows; ++y) {
    const auto* mask_row = mask.ptr<std::uint8_t>(y);
    const auto* depth_row = depth_mm.ptr<float>(y);
    auto* out_row = out.ptr<std::uint8_t>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (mask_row[x] == 0) {
        continue;
      }
      const double depth = depth_row[x];
      if (depth <= 0.0) {
        continue;
      }
      if (std::abs(depth / 1000.0 - median_mm / 1000.0) < threshold_m) {
        out_row[x] = 255;
      }
    }
  }
  return out;
}

cv::Point2d Centroid(const cv::Mat& mask, const BoundingBox& box) {
  // Python 版用 moments(mask) 且掩膜为 0/1，等价于 binaryImage=true 的二值矩。
  const cv::Moments moments = cv::moments(mask, true);
  if (moments.m00 != 0.0) {
    return {moments.m10 / moments.m00, moments.m01 / moments.m00};
  }
  return {0.5 * (box.x1 + box.x2), 0.5 * (box.y1 + box.y2)};
}

bool CentroidDepth(const cv::Mat& depth_mm, const cv::Mat& mask, const cv::Point2d& centroid, int sample_radius,
                   double* depth_mm_out) {
  const auto cX = static_cast<int>(centroid.x);
  const auto cY = static_cast<int>(centroid.y);

  const int x_min = std::max(0, cX - sample_radius);
  const int x_max = std::min(depth_mm.cols, cX + sample_radius);
  const int y_min = std::max(0, cY - sample_radius);
  const int y_max = std::min(depth_mm.rows, cY + sample_radius);
  if (x_max <= x_min || y_max <= y_min) {
    return false;
  }

  std::vector<float> roi_values;
  for (int y = y_min; y < y_max; ++y) {
    const auto* mask_row = mask.ptr<std::uint8_t>(y);
    const auto* depth_row = depth_mm.ptr<float>(y);
    for (int x = x_min; x < x_max; ++x) {
      if (mask_row[x] != 0 && depth_row[x] > 0.0F) {
        roi_values.push_back(depth_row[x]);
      }
    }
  }
  if (roi_values.size() > 5) {
    *depth_mm_out = Median(std::move(roi_values));
    return true;
  }

  // 质心处可能正好落在无效深度（空洞）上，回退到整块掩膜
  return MedianDepth(depth_mm, mask, depth_mm_out);
}

std::vector<cv::Point> ForegroundPixels(const cv::Mat& mask) {
  std::vector<cv::Point> pixels;
  for (int y = 0; y < mask.rows; ++y) {
    const auto* row = mask.ptr<std::uint8_t>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (row[x] != 0) {
        pixels.emplace_back(x, y);
      }
    }
  }
  return pixels;
}

}  // namespace amorph::vision::mask
