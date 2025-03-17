#ifndef ORE_DETECTOR_HPP
#define ORE_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

class OreDetectorNode : public rclcpp::Node {
 public:
  OreDetectorNode(const rclcpp::NodeOptions &options);
  ~OreDetectorNode();

 private:
  void imageCallback(const cv::Mat &frame);
  void processFrame(const cv::Mat &frame);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  cv::VideoCapture cap_;  // VideoCapture object for USB camera
  int lowerB_, upperB_, lowerH_, upperH_, lowerS_, upperS_, lowerV_, upperV_;
  int minContourArea_;
};

#endif  // ORE_DETECTOR_HPP
