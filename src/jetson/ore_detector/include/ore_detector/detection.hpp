#ifndef DETECTION_HPP
#define DETECTION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::Point2f> detectOreCorners(const cv::Mat &frame, int lowerB, int upperB, int lowerH, int upperH,
                                          int lowerS, int upperS, int lowerV, int upperV, int minContourArea);

#endif  // DETECTION_HPP
