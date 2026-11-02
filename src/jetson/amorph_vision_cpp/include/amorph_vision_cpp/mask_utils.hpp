// 掩膜处理：腐蚀、连通域、深度约束、质心。
#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "amorph_vision_cpp/types.hpp"

namespace amorph::vision::mask {

/// 形态学腐蚀。mask 为 CV_8UC1（0/255）。
cv::Mat Erode(const cv::Mat& mask, int kernel_size, int iterations);

/// 只保留面积最大的连通域，返回 CV_8UC1（0/255）；无连通域时返回空 Mat。
cv::Mat LargestComponent(const cv::Mat& binary);

/// mask 内所有有效深度（>0）的中位数，单位毫米；无有效点时返回 false。
bool MedianDepth(const cv::Mat& depth_mm, const cv::Mat& mask, double* median_mm);

/// 用中位数 ±阈值剔除离群深度，返回 CV_8UC1（0/255）。
cv::Mat FilterByDepth(const cv::Mat& depth_mm, const cv::Mat& mask, double median_mm, double threshold_m);

/// 掩膜质心；掩膜为空时回退到检测框中心。
cv::Point2d Centroid(const cv::Mat& mask, const BoundingBox& box);

/// 质心邻域内掩膜覆盖处的深度中位数；邻域点数不足时回退到整块掩膜。
bool CentroidDepth(const cv::Mat& depth_mm, const cv::Mat& mask, const cv::Point2d& centroid, int sample_radius,
                   double* depth_mm_out);

/// 取掩膜内全部像素坐标，返回 (x, y) 对。
std::vector<cv::Point> ForegroundPixels(const cv::Mat& mask);

}  // namespace amorph::vision::mask
