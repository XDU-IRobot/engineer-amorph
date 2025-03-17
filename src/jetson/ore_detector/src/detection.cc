#include "ore_detector/detection.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

std::vector<cv::Point2f> detectOreCorners(const cv::Mat &frame, int lowerB, int upperB, int lowerH, int upperH,
                                          int lowerS, int upperS, int lowerV, int upperV, int minContourArea) {
  std::vector<cv::Point2f> corners;

  cv::Mat hsv, silverMask;
  cv::Mat debug_frame = frame.clone();
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(lowerH, lowerS, lowerV), cv::Scalar(upperH, upperS, upperV), silverMask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(silverMask, silverMask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(silverMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    // cv::drawContours(debug_frame, contours, i, cv::Scalar(0, 0, 255), 2);
    if (cv::contourArea(contours[i]) < minContourArea) {
      continue;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contours[i], approx, 5, true);

    if (approx.size() >= 4 && approx.size() <= 6) {
      cv::Rect boundingRect = cv::boundingRect(approx);
      double aspectRatio = static_cast<double>(boundingRect.width) / boundingRect.height;
      if (aspectRatio > 2.0 || aspectRatio < 0.5) {
        continue;
      }

      cv::Moments m = cv::moments(contours[i]);
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;
      corners.push_back(cv::Point2f(cx, cy));
      cv::circle(debug_frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
    }
  }

  // sort corners 左上 左下 右下 右上
  if (corners.size() == 4) {
    std::sort(corners.begin(), corners.end(),
              [](const cv::Point2f &a, const cv::Point2f &b) { return a.x + a.y < b.x + b.y; });
    if (corners[0].y > corners[1].y) {
      std::swap(corners[0], corners[1]);
    }
    if (corners[2].y < corners[3].y) {
      std::swap(corners[2], corners[3]);
    }
  }

  cv::imshow("Mask", silverMask);
  cv::imshow("debug", debug_frame);
  cv::waitKey(1);

  return corners;
}
